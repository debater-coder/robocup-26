//! Runs a HTTP JSON API server to interact with a simulated RoboCup soccer game.
//! Robots are clients to this interface, and submit commands and receive information
//! about the state of the world.
//!
//! The physics engine simulates the world in ticks which can run faster than real time,
//! and the robot processes have their loops synchronised with these ticks.
//!
//! # Lifecycle
//!
//! Robots first register themselves at /session/register.

use std::{f32::consts::PI, sync::Arc, time::Duration};

use axum::{
    Json, Router,
    extract::State,
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::{delete, get, post},
};
use clap::Parser as _;
use rerun::{
    MediaType, Rotation3D, RotationAxisAngle,
    external::{anyhow, log},
};
use serde::{Deserialize, Serialize};
use simulator::{GameState, RobotSession, Team, Tick};

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
        Err(e) => match e {
            simulator::SessionRegisterError::Conflict { robot_id: _ } => {
                (StatusCode::CONFLICT, Json(e.clone())).into_response()
            }
            simulator::SessionRegisterError::RegisterUnavailable => {
                (StatusCode::SERVICE_UNAVAILABLE, Json(e.clone())).into_response()
            }
        },
    }
}

// TODO
async fn session_delete() {}

async fn world_start(State(state): State<Arc<GameState>>) {
    state.world_start().await;
}

async fn world_tick() {}
async fn robot_command() {}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let (rec, _) = cli.rerun.init("robocup-sim")?;

    rec.log(
        "/sim/field/asset",
        &rerun::Transform3D::from_translation([0., 0., -0.1]),
    );
    rec.log(
        "/sim/field/asset",
        &rerun::Asset3D::from_file_contents(
            include_bytes!("./field.glb").to_vec(),
            Some(MediaType::glb()),
        ),
    )
    .unwrap();

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
