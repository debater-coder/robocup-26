use std::{
    collections::HashMap,
    f32::consts::PI,
    fmt::Display,
    process,
    time::{Duration, Instant},
};

use rapier2d::{na::Isometry2, prelude::*};
use rerun::{FillMode, MediaType, RecordingStream, external::log};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use tokio::sync::{Mutex, watch};

use crate::physics::PhysicsState;

mod physics;

#[derive(Debug, Clone)]
pub struct GameSnapshot {
    pub tick: Tick,
    pub sim_time: Duration,
    pub positions: HashMap<String, Isometry2<f32>>,
    pub velocity: HashMap<String, RigidBodyVelocity<f32>>,
    pub ball_position: Isometry2<f32>,
    pub ball_velocity: RigidBodyVelocity<f32>,
    pub sessions_snapshot: HashMap<String, RobotSession>,
    pub current_phase: GamePhase,
}

pub struct GameState {
    inner: Mutex<GameStateInner>,
    world_tx: watch::Sender<Option<GameSnapshot>>,
    world_rx: watch::Receiver<Option<GameSnapshot>>,
    pub timestep: Duration,
    rec: RecordingStream,
}

pub struct GameStateInner {
    pub tick: Tick,
    pub score: GameScore,
    /// Length of game in ticks
    pub game_length: Tick,
    physics_state: PhysicsState,
    pub sessions: HashMap<String, RobotSession>,
    pub tick_state: TickState,
    pub phase: GamePhase,
}

impl GameState {
    pub fn new(game_length: Duration, rec: RecordingStream) -> Self {
        rec.set_duration_secs("sim_time", 0);
        rec.set_time_sequence("tick", 0);

        rec.log(
            "/sim/field/asset",
            &rerun::Transform3D::from_translation([0., 0., -0.1]),
        )
        .unwrap();
        rec.log(
            "/sim/field/asset",
            &rerun::Asset3D::from_file_contents(
                include_bytes!("./field.glb").to_vec(),
                Some(MediaType::glb()),
            ),
        )
        .unwrap();

        let physics_state = PhysicsState::new(rec.clone());
        let timestep = physics_state.timestep();

        let (tx, rx) = watch::channel(None);

        let game_length = Tick::from_sim_time(&physics_state, game_length);
        GameState {
            inner: Mutex::new(GameStateInner {
                tick: Tick(0),
                score: Default::default(),
                game_length,
                sessions: HashMap::new(),
                tick_state: TickState {
                    current_tick: Tick(0),
                    commands_received: HashMap::new(),
                },
                phase: GamePhase::Kickoff {
                    first_tick: Tick(1),
                },
                physics_state,
            }),
            world_tx: tx,
            world_rx: rx,
            timestep,
            rec,
        }
    }

    /// Begin the first tick (tick 1), starting the simulation and thus broadcasting
    /// the initial world state.
    pub async fn world_start(&self, team: Team) {
        self.position_for_kickoff(team).await;
        self.tick().await;
    }

    /// Waits for the snapshot for a specific tick is received
    pub async fn subscribe_to_game_snapshot(
        &self,
        tick: Tick,
    ) -> Result<GameSnapshot, SubscribeGameSnapshotError> {
        let mut rx = self.world_rx.clone();

        loop {
            if let Some(snapshot) = rx.borrow_and_update().clone() {
                match snapshot.tick.cmp(&tick) {
                    std::cmp::Ordering::Less => {}
                    std::cmp::Ordering::Equal => return Ok(snapshot),
                    std::cmp::Ordering::Greater => {
                        return Err(SubscribeGameSnapshotError::TickExpired(snapshot.tick));
                    }
                }
            };

            match rx.changed().await {
                Ok(()) => {}
                Err(_) => return Err(SubscribeGameSnapshotError::SimulationClosed),
            }
        }
    }

    async fn position_for_kickoff(&self, kickoff_team: Team) {
        let mut inner = self.inner.lock().await;
        log::info!("Positioning for kickoff with team {}", kickoff_team);

        // Center ball
        inner.physics_state.teleport_ball(Vec2::ZERO);

        // Kickoffs alternate
        let non_kickoff = match kickoff_team {
            Team::Cyan => Team::Yellow,
            Team::Yellow => Team::Cyan,
        };

        let kickoff_side = match kickoff_team {
            Team::Cyan => -1.,
            Team::Yellow => 1.,
        };

        let (kickoff_direction, non_kickoff_direction) = match kickoff_team {
            Team::Cyan => (0., PI),
            Team::Yellow => (PI, 0.),
        };

        let non_kickoff_side = -kickoff_side;

        // Position kicking off team
        if let Some(rb) = inner.robot_rb_mut(kickoff_team, 0) {
            rb.set_position(
                Isometry2::new(vector![kickoff_side * 0.2, 0.], kickoff_direction).into(),
                true,
            );
            rb.set_vels(RigidBodyVelocity::zero(), true);
        }
        if let Some(rb) = inner.robot_rb_mut(kickoff_team, 1) {
            rb.set_position(
                Isometry2::new(vector![kickoff_side * 0.615, 0.], kickoff_direction).into(),
                true,
            );
            rb.set_vels(RigidBodyVelocity::zero(), true);
        }

        // Position other side
        if let Some(rb) = inner.robot_rb_mut(non_kickoff, 0) {
            rb.set_position(
                Isometry2::new(
                    vector![non_kickoff_side * 0.615, 0.34],
                    non_kickoff_direction,
                )
                .into(),
                true,
            );
            rb.set_vels(RigidBodyVelocity::zero(), true);
        }
        if let Some(rb) = inner.robot_rb_mut(non_kickoff, 1) {
            rb.set_position(
                Isometry2::new(
                    vector![non_kickoff_side * 0.615, -0.34],
                    non_kickoff_direction,
                )
                .into(),
                true,
            );
            rb.set_vels(RigidBodyVelocity::zero(), true);
        }
    }

    /// Advances to next tick, sending the snapshot of the
    /// world on the channel as it is currently. This assumes
    /// all commands are already applied.
    async fn tick(&self) {
        let mut inner = self.inner.lock().await;

        inner.tick.0 += 1;

        if inner.tick >= inner.game_length {
            let _ = self.rec.flush_blocking();
            process::exit(0);
        }

        inner.tick_state = TickState {
            current_tick: inner.tick,
            commands_received: HashMap::new(),
        };

        let tick = inner.tick;
        inner.phase.advance(tick);

        self.rec
            .set_time("sim_time", inner.tick.to_sim_time(&inner.physics_state));
        self.rec.set_time_sequence("tick", inner.tick.0);

        let elapsed = inner.tick.to_sim_time(&inner.physics_state);
        self.rec
            .log(
                "/sim/status",
                &rerun::TextDocument::from_markdown(format!(
                    r#"
| **Game Phase**     | `{:?}` |
| ---                | ---    |
| **Cyan score**     | {}     |
| **Yellow score**   | {}     |
| **Game clock**     | {}:{}     |
            "#,
                    inner.phase,
                    inner.score.cyan,
                    inner.score.yellow,
                    elapsed.as_secs() / 60,
                    elapsed.as_secs() % 60
                )),
            )
            .unwrap();

        for (robot_id, robot) in inner.sessions.iter() {
            let pos = inner
                .physics_state
                .get_rb(robot.rigid_body)
                .unwrap()
                .position();

            self.rec
                .log(
                    format!("/sim/robots/{}", robot_id),
                    &rerun::Cylinders3D::from_lengths_and_radii([0.22], [0.11])
                        .with_centers([(pos.translation.x, pos.translation.y, 0.11)])
                        .with_colors([match robot.team {
                            Team::Cyan => (0, 255, 255),
                            Team::Yellow => (255, 255, 0),
                        }])
                        .with_fill_mode(FillMode::Solid),
                )
                .unwrap();

            self.rec
                .log(
                    format!("/sim/robots/{}", robot_id),
                    &rerun::Arrows3D::from_vectors([(
                        pos.rotation.re * 0.2,
                        pos.rotation.im * 0.2,
                        0.,
                    )])
                    .with_origins([(pos.translation.x, pos.translation.y, 0.11)])
                    .with_radii([0.01]),
                )
                .unwrap();
        }

        let ball_pos = inner.physics_state.get_ball_position().translation;

        self.rec
            .log(
                "/sim/ball",
                &rerun::Points3D::new([(ball_pos.x, ball_pos.y, 0.042 / 2.)])
                    .with_radii([0.021])
                    .with_colors([(255, 165, 0)]),
            )
            .unwrap();

        let _ = self.world_tx.send(Some(GameSnapshot {
            tick: inner.tick,
            sim_time: inner.tick.to_sim_time(&inner.physics_state),
            positions: inner
                .sessions
                .iter()
                .map(|(k, v)| {
                    (
                        k.clone(),
                        (*inner.physics_state.get_rb(v.rigid_body).unwrap().position()).into(),
                    )
                })
                .collect(),
            velocity: inner
                .sessions
                .iter()
                .map(|(k, v)| {
                    (
                        k.clone(),
                        *inner.physics_state.get_rb(v.rigid_body).unwrap().vels(),
                    )
                })
                .collect(),
            ball_position: inner.physics_state.get_ball_position().into(),
            ball_velocity: inner.physics_state.get_ball_velocity(),
            sessions_snapshot: inner.sessions.clone(),
            current_phase: inner.phase.clone(),
        }));
    }

    /// Processes a command from the robot process, applying them to the world
    /// and advancing to the next tick once all commands are received.
    pub async fn insert_command(
        &self,
        robot_id: String,
        tick: Tick,
        command: Option<RobotCommand>,
    ) -> Result<(), InsertCommandError> {
        // All operations involving the inner lock are within a block to prevent deadlock
        let (should_tick, goal_scored) = {
            let mut inner = self.inner.lock().await;

            if inner.tick_state.current_tick != tick {
                return Err(InsertCommandError::TickNotCurrent(tick));
            }

            let Some(session) = inner.sessions.get(&robot_id) else {
                return Err(InsertCommandError::RobotNotFound(robot_id));
            };

            if command.is_some()
                && !(inner.phase == GamePhase::Playing && session.status == RobotStatus::Active)
            {
                return Err(InsertCommandError::CannotMove);
            }

            inner.tick_state.commands_received.insert(robot_id, command);

            if inner
                .sessions
                .keys()
                .all(|key| inner.tick_state.commands_received.contains_key(key))
            {
                // All commands now received, apply to physics engine
                for (robot_id, command) in inner.tick_state.commands_received.clone().iter() {
                    if let Some(command) = command {
                        let rb = inner.sessions.get(robot_id).unwrap().rigid_body;
                        let rb = inner.physics_state.get_rb_mut(rb).unwrap();

                        // Rotate velocity from robot coordinate space into world-space
                        let vel = rb
                            .rotation()
                            .transform_vector(Vec2::new(command.vx, command.vy));

                        self.rec
                            .log(
                                format!("/sim/robots/{}/vel", robot_id),
                                &rerun::Arrows3D::from_vectors([(vel.x, vel.y, 0.)])
                                    .with_radii([0.01])
                                    .with_origins([(rb.translation().x, rb.translation().y, 0.)]),
                            )
                            .unwrap();

                        rb.apply_impulse(vel * rb.mass(), true);
                        rb.apply_torque_impulse(command.omega * rb.mass(), true);
                    }
                }

                // Run a step of the physics engine
                let mut scored = None; // The team scored against
                for event in inner.physics_state.step() {
                    match event {
                        physics::PhysicsStateEvent::BallHitGoal(team) => scored = Some(team),
                    }
                }

                match scored {
                    Some(Team::Cyan) => inner.score.yellow += 1,
                    Some(Team::Yellow) => inner.score.cyan += 1,
                    _ => {}
                }

                if scored.is_some() {
                    inner.phase = GamePhase::Kickoff {
                        first_tick: Tick(inner.tick.0 + 1),
                    };
                }

                // New tick
                (true, scored)
            } else {
                (false, None)
            }
        };

        if let Some(team) = goal_scored {
            self.position_for_kickoff(team).await;
        }

        if should_tick {
            self.tick().await;
        }

        Ok(())
    }

    /// Registers a new robot session. Only works during tick 0.
    pub async fn register_session(
        &self,
        team: Team,
        robot_index: usize,
    ) -> Result<RobotSession, SessionRegisterError> {
        let robot_id = RobotSession::get_robot_id(team, robot_index);

        let mut inner = self.inner.lock().await;

        if inner.tick != Tick(0) {
            return Err(SessionRegisterError::RegisterUnavailable);
        }

        if let Some(_) = inner.sessions.get(&robot_id) {
            Err(SessionRegisterError::Conflict { robot_id })
        } else {
            let session = RobotSession::new(
                robot_id.clone(),
                team,
                robot_index,
                inner.physics_state.spawn_robot(),
            );

            inner.sessions.insert(robot_id, session.clone());

            log::info!(
                "Registered session for robot {}, {:?}",
                &session.robot_id,
                &session
            );

            Ok(session)
        }
    }
}

impl GameStateInner {
    fn robot_rb_mut(&mut self, team: Team, robot_index: usize) -> Option<&mut RigidBody> {
        self.sessions
            .get(&RobotSession::get_robot_id(team, robot_index))
            .and_then(|session| self.physics_state.get_rb_mut(session.rigid_body))
    }
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub enum GamePhase {
    /// Kickoffs freeze robots in their kickoff positions and last 5 ticks
    Kickoff {
        /// The first frozen tick
        first_tick: Tick,
    },
    Playing,
}

impl GamePhase {
    fn advance(&mut self, current_tick: Tick) {
        *self = match &self {
            GamePhase::Kickoff { first_tick } => {
                // During the last tick, we set the game phase to playing so that clients
                // can submit their command for the following tick
                if current_tick.0 >= first_tick.0 + 4 {
                    GamePhase::Playing
                } else {
                    self.clone()
                }
            }
            GamePhase::Playing => GamePhase::Playing,
        };
    }
}

#[derive(Debug, Default)]
pub struct GameScore {
    cyan: u32,
    yellow: u32,
}

/// Which goal a robot is defending
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Team {
    Cyan,
    Yellow,
}

impl Display for Team {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Team::Cyan => "cyan",
                Team::Yellow => "yellow",
            }
        )
    }
}

/// A tick in simulated time. Every tick, robots submit their commands,
/// the physics engine runs, and then the world state is queryable by
/// the robots.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, PartialOrd, Ord)]
pub struct Tick(pub u32);

impl Display for Tick {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "tick({})", self.0)
    }
}

/// Holds pending state for the next tick
pub struct TickState {
    current_tick: Tick,
    /// Pending commands to be applied to the world for the next tick
    /// If robots are unable to move (kickoff or damaged), they must still submit a None
    /// command to keep robot loop synchronised with sim time.
    commands_received: HashMap<String, Option<RobotCommand>>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RobotCommand {
    /// (m/s)
    pub vx: f32,
    /// (m/s)
    pub vy: f32,
    /// (rad/s)
    pub omega: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
pub enum RobotStatus {
    Active,
    /// Marked as damaged; pending removal from field
    Damaged(DamageReason),
    /// Removed from field for next 30s (5.7.2)
    Removed {
        removed_at: Tick,
        damage_reason: DamageReason,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
pub enum DamageReason {
    NotResponding,
    /// Rule 5.7.1.2: Remaining in goal area for more than 20s
    StayingNearGoal,
    /// Rule 5.7.1.6: Whole of robot entered out area (and not pushed by other robot)
    /// Pushing is determined using dot product of target velocity and actual velocity
    EnteredOutArea,
}

#[derive(Debug, Clone, Serialize)]
pub struct RobotSession {
    pub robot_id: String,
    pub team: Team,
    /// Index of robot within the team
    pub robot_index: usize,
    #[serde(skip)]
    pub registered_at: Instant,
    #[serde(skip)]
    pub last_seen_at: Instant,
    pub status: RobotStatus,
    pub rigid_body: RigidBodyHandle,
}

impl RobotSession {
    pub fn get_robot_id(team: Team, robot_index: usize) -> String {
        format!("r_{}_{}", team, robot_index)
    }

    fn new(robot_id: String, team: Team, robot_index: usize, rigid_body: RigidBodyHandle) -> Self {
        Self {
            robot_id,
            team,
            robot_index,
            registered_at: Instant::now(),
            last_seen_at: Instant::now(),
            status: RobotStatus::Active,
            rigid_body,
        }
    }
}

impl Tick {
    /// Converts a duration in simulated time to a number of ticks
    fn from_sim_time(physics_state: &PhysicsState, duration: Duration) -> Tick {
        Tick((duration.as_nanos() / physics_state.timestep().as_nanos()) as u32)
    }

    /// Converts a number of ticks to a duration in simulated time
    fn to_sim_time(&self, physics_state: &PhysicsState) -> Duration {
        physics_state.timestep() * self.0
    }
}

#[derive(Error, Debug, Serialize, Clone)]
pub enum SessionRegisterError {
    #[error("robot session for {robot_id} already exists")]
    Conflict { robot_id: String },
    #[error("robot session cannot be registered after first tick")]
    RegisterUnavailable,
}

#[derive(Error, Debug, Serialize, Clone)]
pub enum SubscribeGameSnapshotError {
    #[error("tick {0} already passed")]
    TickExpired(Tick),
    #[error("simulation channel closed")]
    SimulationClosed,
}

#[derive(Error, Debug, Serialize, Clone)]
pub enum InsertCommandError {
    #[error("tick {0} is not current tick")]
    TickNotCurrent(Tick),
    #[error("robot_id {0} not found")]
    RobotNotFound(String),
    #[error("moving not allowed")]
    CannotMove,
}
