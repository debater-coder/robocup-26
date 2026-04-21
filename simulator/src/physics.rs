use std::{sync::mpsc::channel, time::Duration};

use rapier2d::prelude::*;
use rerun::RecordingStream;

use crate::Team;

pub(crate) struct PhysicsState {
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
    physics_pipeline: PhysicsPipeline,
    event_handler: ChannelEventCollector,
    collision_recv: std::sync::mpsc::Receiver<CollisionEvent>,
    contact_force_recv: std::sync::mpsc::Receiver<ContactForceEvent>,
    goals: Goals,
    ball: RigidBodyHandle,
}

struct Goals {
    cyan: ColliderHandle,
    yellow: ColliderHandle,
}

pub(crate) enum PhysicsStateEvent {
    BallHitGoal(Team),
}

impl PhysicsState {
    pub(crate) fn new(rec: RecordingStream) -> Self {
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        rec.log(
            "/sim/field",
            &rerun::Boxes3D::from_centers_and_sizes([(0., 0., 0.11)], [(2.43, 1.82, 0.22)])
                .with_colors([(0, 255, 0)]),
        )
        .unwrap();

        let positions: [(f32, f32); 4] = [
            (2.43 / 2. + 0.1, 0.),
            (-(2.43 / 2. + 0.1), 0.),
            (0., 1.82 / 2. + 0.1),
            (0., -(1.82 / 2. + 0.1)),
        ];

        let sizes = [(0.2, 1.82), (0.2, 1.82), (2.43, 0.2), (2.43, 0.2)];

        for (position, size) in positions.iter().zip(sizes.iter()) {
            let rb = RigidBodyBuilder::fixed()
                .translation(Vector::from(position.clone()))
                .build();
            let rb = rigid_body_set.insert(rb);

            let collider = ColliderBuilder::cuboid(size.0 / 2., size.1 / 2.).build();

            collider_set.insert_with_parent(collider, rb, &mut rigid_body_set);
        }

        rec.log(
            "/sim/field/walls",
            &rerun::Boxes3D::from_centers_and_sizes(
                positions.map(|pos| (pos.0, pos.1, 0.11)),
                sizes.map(|size| (size.0, size.1, 0.22)),
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

        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 0.05; // 20 ticks per second

        let (collision_send, collision_recv) = channel();
        let (contact_force_send, contact_force_recv) = channel();
        PhysicsState {
            integration_parameters,
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            ball: PhysicsState::spawn_ball(&mut collider_set, &mut rigid_body_set),
            goals: PhysicsState::spawn_goals(&mut collider_set, rec.clone()),
            rigid_body_set,
            collider_set,
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_pipeline: PhysicsPipeline::new(),
            event_handler: ChannelEventCollector::new(collision_send, contact_force_send),
            collision_recv,
            contact_force_recv,
            rec,
        }
    }

    /// Returns of the timestep simulated every tick
    pub(crate) fn timestep(&self) -> Duration {
        Duration::from_secs_f32(self.integration_parameters.dt)
    }

    pub(crate) fn spawn_robot(&mut self) -> RigidBodyHandle {
        let rb = RigidBodyBuilder::dynamic()
            // .linear_damping(2.0)
            // .angular_damping(5.0)
            .build();
        let rb = self.rigid_body_set.insert(rb);

        let collider = ColliderBuilder::ball(0.110).mass(2.5).build();

        self.collider_set
            .insert_with_parent(collider, rb, &mut self.rigid_body_set);

        rb
    }

    pub(crate) fn get_rb_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.rigid_body_set.get_mut(handle)
    }

    fn spawn_ball(colliders: &mut ColliderSet, bodies: &mut RigidBodySet) -> RigidBodyHandle {
        let rb = RigidBodyBuilder::dynamic().build();
        let rb = bodies.insert(rb);
        let collider = ColliderBuilder::ball(0.021)
            .mass(0.046)
            .restitution(0.9)
            .build();
        colliders.insert_with_parent(collider, rb, bodies);
        rb
    }

    pub(crate) fn teleport_ball(&mut self, coords: Vector) {
        let rb = self.get_rb_mut(self.ball).unwrap();

        rb.set_translation(coords, true);
        rb.set_vels(RigidBodyVelocity::zero(), true);
    }

    pub(crate) fn get_ball_position(&self) -> Pose2 {
        *self.get_rb(self.ball).unwrap().position()
    }

    pub(crate) fn get_ball_velocity(&self) -> RigidBodyVelocity<f32> {
        *self.get_rb(self.ball).unwrap().vels()
    }

    pub(crate) fn get_rb(&self, rigid_body: RigidBodyHandle) -> Option<&RigidBody> {
        self.rigid_body_set.get(rigid_body)
    }

    pub(crate) fn step(&mut self) -> impl Iterator<Item = PhysicsStateEvent> {
        self.physics_pipeline.step(
            Vector2::ZERO,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &self.event_handler,
        );

        self.collision_recv
            .try_iter()
            .filter_map(|event| match event {
                CollisionEvent::Started(h1, h2, _) => {
                    let ball_collider = self.get_rb(self.ball).unwrap().colliders()[0];
                    let other_handle = {
                        if h1 == ball_collider {
                            Some(h2)
                        } else if h2 == ball_collider {
                            Some(h1)
                        } else {
                            None
                        }
                    };

                    match other_handle {
                        Some(handle) if handle == self.goals.cyan => {
                            Some(PhysicsStateEvent::BallHitGoal(Team::Cyan))
                        }
                        Some(handle) if handle == self.goals.yellow => {
                            Some(PhysicsStateEvent::BallHitGoal(Team::Yellow))
                        }
                        _ => None,
                    }
                }
                _ => None,
            })
    }

    fn spawn_goals(colliders: &mut ColliderSet, rec: RecordingStream) -> Goals {
        rec.log(
            "/sim/field/goals/cyan",
            &rerun::Boxes3D::from_centers_and_sizes(
                [(-(1.83 + 0.074) / 2., 0., 0.07)],
                [(0.074, 0.45, 0.14)],
            )
            .with_colors([(0, 255, 255)]),
        )
        .unwrap();

        rec.log(
            "/sim/field/goals/yellow",
            &rerun::Boxes3D::from_centers_and_sizes(
                [((1.83 + 0.074) / 2., 0., 0.07)],
                [(0.074, 0.45, 0.14)],
            )
            .with_colors([(255, 255, 0)]),
        )
        .unwrap();

        let cyan = ColliderBuilder::cuboid(0.074 / 2., 0.45 / 2.)
            .sensor(true)
            .translation(Vec2::new(-(1.83 + 0.074) / 2., 0.))
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build();

        let yellow = ColliderBuilder::cuboid(0.074 / 2., 0.45 / 2.)
            .sensor(true)
            .translation(Vec2::new((1.83 + 0.074) / 2., 0.))
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build();

        Goals {
            cyan: colliders.insert(cyan),
            yellow: colliders.insert(yellow),
        }
    }
}
