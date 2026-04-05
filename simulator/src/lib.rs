use std::{
    collections::HashMap,
    fmt::Display,
    time::{Duration, Instant},
};

use rapier2d::{
    na::{Isometry, Isometry2},
    prelude::*,
};
use rerun::{FillMode, RecordingStream, external::log};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use tokio::sync::{Mutex, watch};

pub struct GameSnapshot {}

pub struct GameState {
    inner: Mutex<GameStateInner>,
    watch_tx: watch::Sender<Option<GameSnapshot>>,
    pub timestep: Duration,
    rec: RecordingStream,
}

pub struct GameStateInner {
    pub tick: Tick,
    pub score: GameScore,
    /// Length of game in ticks
    pub game_length: Tick,
    pub next_kickoff: Team,
    physics_state: PhysicsState,
    pub sessions: HashMap<String, RobotSession>,
    pub tick_state: TickState,
    phase: GamePhase,
    ball: RigidBodyHandle,
}

impl GameStateInner {
    fn robot_rb_mut(&mut self, team: Team, robot_index: usize) -> Option<&mut RigidBody> {
        self.sessions
            .get(&RobotSession::get_robot_id(team, robot_index))
            .and_then(|session| self.physics_state.get_rb_mut(session.rigid_body))
    }
}

pub enum GamePhase {
    /// Kickoffs freeze robots in their kickoff positions and last 5 ticks
    Kickoff {
        /// The first frozen tick
        first_tick: Tick,
    },
    Playing,
}

struct PhysicsState {
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    rec: RecordingStream,
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Tick(pub u32);

/// Holds pending state for the next tick
pub struct TickState {
    pub current_tick: Tick,
    /// Pending commands to be applied to the world for the next tick
    pub commands_received: HashMap<String, RobotCommand>,
}

pub struct RobotCommand {
    /// Velocity forward in robot coordinate frame (m/s)
    pub vx: f32,
    /// Velocity left in robot coordinate frame (m/s)
    pub vy: f32,
    /// Rotational velocity anticlockwise (rad/s)
    pub omega: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DamageReason {
    NotResponding,
    /// Rule 5.7.1.2: Remaining in goal area for more than 20s
    StayingNearGoal,
    /// Rule 5.7.1.6: Whole of robot entered out area (and not pushed by other robot)
    /// Pushing is determined using dot product of target velocity and actual velocity
    EnteredOutArea,
}

#[derive(Debug, Clone)]
pub struct RobotSession {
    pub robot_id: String,
    pub team: Team,
    /// Index of robot within the team
    pub robot_index: usize,
    pub registered_at: Instant,
    pub last_seen_at: Instant,
    pub status: RobotStatus,
    pub rigid_body: RigidBodyHandle,
}

impl RobotSession {
    fn get_robot_id(team: Team, robot_index: usize) -> String {
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

impl PhysicsState {
    fn new(rec: RecordingStream) -> Self {
        rec.log(
            "/sim/field",
            &rerun::Boxes3D::from_centers_and_sizes([(0., 0., 0.11)], [(2.43, 1.82, 0.22)])
                .with_colors([(0, 255, 0)]),
        )
        .unwrap();

        rec.log(
            "/sim/field/walls",
            &rerun::Boxes3D::from_centers_and_sizes(
                [
                    (2.43 / 2. + 0.1, 0., 0.11),
                    (-(2.43 / 2. + 0.1), 0., 0.11),
                    (0., 1.82 / 2. + 0.1, 0.11),
                    (0., -(1.82 / 2. + 0.1), 0.11),
                ],
                [
                    (0.2, 1.82, 0.22),
                    (0.2, 1.82, 0.22),
                    (2.43, 0.2, 0.22),
                    (2.43, 0.2, 0.22),
                ],
            ),
        )
        .unwrap();

        rec.log(
            "/sim/field/penalty_boxes/cyan",
            &rerun::Boxes3D::from_centers_and_sizes(
                [(-(1.93 - 0.1) / 2. + 0.15, 0., 0.11)],
                [(0.3, 0.9, 0.22)],
            ),
        )
        .unwrap();

        rec.log(
            "/sim/field/penalty_boxes/yellow",
            &rerun::Boxes3D::from_centers_and_sizes(
                [((1.93 - 0.1) / 2. - 0.15, 0., 0.11)],
                [(0.3, 0.9, 0.22)],
            ),
        )
        .unwrap();

        rec.log(
            "/sim/field/bounds",
            &rerun::Boxes3D::from_centers_and_sizes([(0., 0., 0.11)], [(1.93, 1.32, 0.22)])
                .with_colors([(255, 255, 255)]),
        )
        .unwrap();

        rec.log(
            "/sim/field/bounds_inner",
            &rerun::Boxes3D::from_centers_and_sizes(
                [(0., 0., 0.11)],
                [(1.93 - 0.1, 1.32 - 0.1, 0.22)],
            )
            .with_colors([(255, 255, 255)]),
        )
        .unwrap();

        PhysicsState {
            integration_parameters: IntegrationParameters::default(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            rec,
        }
    }

    /// Returns of the timestep simulated every tick
    fn timestep(&self) -> Duration {
        Duration::from_secs_f32(self.integration_parameters.dt)
    }

    fn spawn_robot(&mut self) -> RigidBodyHandle {
        let rb = RigidBodyBuilder::dynamic().build();
        let rb = self.rigid_body_set.insert(rb);

        let collider = ColliderBuilder::ball(0.220).mass(2.5).build();

        self.collider_set
            .insert_with_parent(collider, rb, &mut self.rigid_body_set);

        rb
    }

    fn get_rb_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.rigid_body_set.get_mut(handle)
    }

    fn spawn_ball(&mut self) -> RigidBodyHandle {
        let rb = RigidBodyBuilder::dynamic().build();
        let rb = self.rigid_body_set.insert(rb);
        let collider = ColliderBuilder::ball(0.021).mass(0.046).build();
        self.collider_set
            .insert_with_parent(collider, rb, &mut self.rigid_body_set);
        rb
    }

    fn get_rb(&self, rigid_body: RigidBodyHandle) -> Option<&RigidBody> {
        self.rigid_body_set.get(rigid_body)
    }
}

#[derive(Error, Debug, Serialize, Clone)]
pub enum SessionRegisterError {
    #[error("robot session for {robot_id} already exists")]
    Conflict { robot_id: String },
    #[error("robot session cannot be registered after first tick")]
    RegisterUnavailable,
}

impl GameState {
    pub fn new(kickoff: Team, game_length: Duration, rec: RecordingStream) -> Self {
        rec.set_duration_secs("sim_time", 0);
        rec.set_time_sequence("tick", 0);

        let mut physics_state = PhysicsState::new(rec.clone());
        let timestep = physics_state.timestep();

        GameState {
            inner: Mutex::new(GameStateInner {
                tick: Tick(0),
                score: Default::default(),
                game_length: Tick::from_sim_time(&physics_state, game_length),
                next_kickoff: kickoff,
                sessions: HashMap::new(),
                tick_state: TickState {
                    current_tick: Tick(0),
                    commands_received: HashMap::new(),
                },
                phase: GamePhase::Kickoff {
                    first_tick: Tick(1),
                },
                ball: physics_state.spawn_ball(),
                physics_state,
            }),
            watch_tx: watch::channel(None).0,
            timestep,
            rec,
        }
    }

    /// Begin the first tick (tick 1), starting the simulation and thus broadcasting
    /// the initial world state.
    pub async fn world_start(&self) {
        self.position_for_kickoff().await;
        self.tick().await;
    }

    async fn position_for_kickoff(&self) {
        let mut inner = self.inner.lock().await;
        let kickoff_team = inner.next_kickoff;
        log::info!("Positioning for kickoff with team {}", kickoff_team);

        // Center ball
        let ball_rb = inner.ball;
        inner
            .physics_state
            .get_rb_mut(ball_rb)
            .unwrap()
            .set_translation(vector![0., 0.].into(), true);

        // Kickoffs alternate
        let non_kickoff = match kickoff_team {
            Team::Cyan => Team::Yellow,
            Team::Yellow => Team::Cyan,
        };

        inner.next_kickoff = non_kickoff;

        let kickoff_side = match kickoff_team {
            Team::Cyan => -1.,
            Team::Yellow => 1.,
        };

        let non_kickoff_side = -kickoff_side;

        // Position kicking off team
        if let Some(rb) = inner.robot_rb_mut(kickoff_team, 0) {
            rb.set_position(
                Isometry2::new(vector![kickoff_side * 0.2, 0.], 0.).into(),
                true,
            );
        }
        if let Some(rb) = inner.robot_rb_mut(kickoff_team, 1) {
            rb.set_position(
                Isometry2::new(vector![kickoff_side * 0.615, 0.], 0.).into(),
                true,
            );
        }

        // Position other side
        if let Some(rb) = inner.robot_rb_mut(non_kickoff, 0) {
            rb.set_position(
                Isometry2::new(vector![non_kickoff_side * 0.615, 0.34], 0.).into(),
                true,
            );
        }
        if let Some(rb) = inner.robot_rb_mut(non_kickoff, 1) {
            rb.set_position(
                Isometry2::new(vector![non_kickoff_side * 0.615, -0.34], 0.).into(),
                true,
            );
        }
    }

    /// Advances to next tick, sending the snapshot of the
    /// world on the channel as it is currently. This assumes
    /// all commands are already applied.
    async fn tick(&self) {
        let mut inner = self.inner.lock().await;

        inner.tick.0 += 1;

        inner.tick_state = TickState {
            current_tick: inner.tick,
            commands_received: HashMap::new(),
        };

        self.rec
            .set_time("sim_time", inner.tick.to_sim_time(&inner.physics_state));
        self.rec.set_time_sequence("tick", inner.tick.0);

        for (robot_id, robot) in inner.sessions.iter() {
            let pos = inner
                .physics_state
                .get_rb(robot.rigid_body)
                .unwrap()
                .translation();

            self.rec
                .log(
                    format!("/sim/robots/{}", robot_id),
                    &rerun::Cylinders3D::from_lengths_and_radii([0.22], [0.11])
                        .with_centers([(pos.x, pos.y, 0.11)])
                        .with_colors([match robot.team {
                            Team::Cyan => (0, 255, 255),
                            Team::Yellow => (255, 255, 0),
                        }])
                        .with_fill_mode(FillMode::Solid),
                )
                .unwrap();
        }

        let ball = inner.ball;
        let ball_pos = inner.physics_state.get_rb(ball).unwrap().translation();

        self.rec
            .log(
                "/sim/ball",
                &rerun::Points3D::new([(ball_pos.x, ball_pos.y, 0.042 / 2.)])
                    .with_radii([0.021])
                    .with_colors([(255, 165, 0)]),
            )
            .unwrap();
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
