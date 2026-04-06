//! Runs a HTTP JSON API server to interact with a simulated RoboCup soccer game.
//! Robots are clients to this interface, and submit commands and receive information
//! about the state of the world.
//!
//! The physics engine simulates the world in ticks which can run faster than real time,
//! and the robot processes have their loops synchronised with these ticks.
//!
//! # Lifecycle
//!
//! 1. Robots first register themselves at /session/register.
//! 2. Robots request /world/tick?robot_id=X&tick=1
//! 3. /world/start is called
//! --- START OF TICK 1 ---
//! 4. Response at /world/tick is sent, can_move=false
//! 5. Robots submit empty commands at /robot/command, then request /world/tick?robot_id=X&tick=2
//! 6. Commands applied, physics engine runs
//! --- START OF TICK 2 ---
//! 7. Robots receive response with new world snapshot, can_move=false
//! ...
//! --- START OF TICK 5 ---
//! 8. Response at /world/tick?robot_id=X&tick=5 is sent, can_move=true (since can_move is for commands applying to tick 6)
//! 9. Robots submit real commands at /robot/command, then request /world/tick?robot_id=X&tick=6
//! 10. Commands get applied to rigid bodies as forces, physics engine runs
//! --- START OF TICK 6 ---
//! 11. Robots receive response at /world/tick with new world snapshot, can_move=true

use std::{sync::Arc, time::Duration};

use axum::{
    Json, Router,
    extract::{Query, State},
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::{delete, get, post},
};
use clap::Parser as _;
use rerun::external::{anyhow, log};
use serde::{Deserialize, Serialize};
use serde_json::json;
use simulator::{
    GamePhase, GameSnapshot, GameState, RobotCommand, RobotSession, RobotStatus,
    SubscribeGameSnapshotError, Team, Tick,
};

#[derive(Debug, clap::Parser)]
#[clap(about = "Runs a simulator server.", long_about = None)]
struct Cli {
    #[command(flatten)]
    rerun: rerun::clap::RerunArgs,
}

#[derive(Debug, Deserialize)]
struct SessionRegisterInfo {
    team: Team,
    robot_index: usize,
}

#[derive(Debug, Serialize)]
struct SessionRegisterResponse {
    robot_id: String,
    team: Team,
    robot_index: usize,
    timestep: Duration,
    next_tick: Tick,
}

async fn session_register(
    State(state): State<Arc<GameState>>,
    Json(SessionRegisterInfo { team, robot_index }): Json<SessionRegisterInfo>,
) -> Response {
    match state.register_session(team, robot_index).await {
        Ok(RobotSession {
            robot_id,
            team,
            robot_index,
            registered_at: _,
            last_seen_at: _,
            status: _,
            rigid_body: _,
        }) => (
            StatusCode::OK,
            Json(SessionRegisterResponse {
                robot_id,
                team,
                robot_index,
                timestep: state.timestep,
                next_tick: Tick(1),
            }),
        )
            .into_response(),
        Err(e) => {
            log::error!("{:?}", e.clone());
            match e {
                simulator::SessionRegisterError::Conflict { robot_id: _ } => {
                    (StatusCode::CONFLICT, Json(e.clone())).into_response()
                }
                simulator::SessionRegisterError::RegisterUnavailable => {
                    (StatusCode::SERVICE_UNAVAILABLE, Json(e.clone())).into_response()
                }
            }
        }
    }
}

// TODO
async fn session_delete() {}

async fn world_start(State(state): State<Arc<GameState>>) {
    state.world_start().await;
}

#[derive(Debug, Deserialize)]
struct WorldTickQueryParams {
    robot_id: String,
    tick: Tick,
}
/// Gets the world snapshot at the start of the requested tick.
/// All coordinates in world frame.
async fn world_tick(
    State(state): State<Arc<GameState>>,
    Query(params): Query<WorldTickQueryParams>,
) -> Response {
    match state.subscribe_to_game_snapshot(params.tick).await {
        Ok(GameSnapshot {
            tick,
            sim_time,
            positions,
            velocity,
            ball_position,
            ball_velocity,
            sessions_snapshot,
            current_phase,
        }) => {
            let Some(self_robot) = sessions_snapshot.get(&params.robot_id) else {
                return (
                    StatusCode::NOT_FOUND,
                    format!("Could not find session for robot {}", params.robot_id),
                )
                    .into_response();
            };

            let can_move =
                current_phase == GamePhase::Playing && self_robot.status == RobotStatus::Active;

            let teammate_id = RobotSession::get_robot_id(
                self_robot.team,
                if self_robot.robot_index == 0 { 1 } else { 0 },
            );

            let opposing_team = match self_robot.team {
                Team::Cyan => Team::Yellow,
                Team::Yellow => Team::Cyan,
            };

            (
                StatusCode::OK,
                Json(json!({
                    "tick": tick,
                    "sim_time": sim_time,
                    "can_move": can_move,
                    "self": {
                        "pose": positions.get(&params.robot_id),
                        "velocity": velocity.get(&params.robot_id),
                    },
                    "teammate": {
                        "pose": positions.get(&teammate_id),
                        "velocity": velocity.get(&teammate_id),
                    },
                    "opponents": [
                        {
                            "pose": positions.get(&RobotSession::get_robot_id(opposing_team, 0)),
                            "velocity": velocity.get(&RobotSession::get_robot_id(opposing_team, 0)),
                        },
                        {
                            "pose": positions.get(&RobotSession::get_robot_id(opposing_team, 1)),
                            "velocity": velocity.get(&RobotSession::get_robot_id(opposing_team, 1)),
                        }
                    ],
                    "ball": {
                        "pose": ball_position,
                        "velocity": ball_velocity
                    }
                })),
            )
                .into_response()
        }
        Err(e) => {
            log::error!("{:?}", e.clone());
            match e {
                SubscribeGameSnapshotError::TickExpired(_) => (StatusCode::BAD_REQUEST, Json(e)),
                SubscribeGameSnapshotError::SimulationClosed => {
                    (StatusCode::SERVICE_UNAVAILABLE, Json(e))
                }
            }
            .into_response()
        }
    }
}

#[derive(Debug, Deserialize)]
struct RobotCommandBody {
    robot_id: String,
    tick: Tick,
    command: Option<RobotCommand>,
}

async fn robot_command(
    State(state): State<Arc<GameState>>,
    Json(RobotCommandBody {
        robot_id,
        tick,
        command,
    }): Json<RobotCommandBody>,
) -> Response {
    match state.insert_command(robot_id, tick, command).await {
        Ok(()) => (
            StatusCode::OK,
            Json(json!({
                "accepted": true,
                "tick": tick
            })),
        )
            .into_response(),
        Err(e) => {
            log::error!("{:?}", e.clone());
            (
                match e {
                    simulator::InsertCommandError::RobotNotFound(_) => StatusCode::NOT_FOUND,
                    simulator::InsertCommandError::CannotMove
                    | simulator::InsertCommandError::TickNotCurrent(_) => StatusCode::BAD_REQUEST,
                },
                Json(e),
            )
                .into_response()
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let (rec, _) = cli.rerun.init("robocup-sim")?;

    rerun::Logger::new(rec.clone())
        .with_path_prefix("/sim/log")
        .init()?;

    let state = Arc::new(GameState::new(
        Team::Yellow,
        Duration::from_mins(10),
        rec.clone(),
    ));

    let app = Router::new()
        .route("/session/register", post(session_register))
        .route("/session/{robot_id}", delete(session_delete))
        .route("/world/tick", get(world_tick))
        .route("/world/start", post(world_start))
        .route("/robot/command", post(robot_command))
        .with_state(state);

    println!("Listening on http://0.0.0.0:3000");
    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await?;
    axum::serve(listener, app).await?;

    log::logger().flush();
    Ok(())
}
