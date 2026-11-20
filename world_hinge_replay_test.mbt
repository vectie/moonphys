///|
test "rigid body world resolves hinge constraints on motion bodies" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let root_state = @moonphys.rigid_body_motion_state(
    "root",
    2.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let upper_state = @moonphys.rigid_body_motion_state(
    "upper",
    2.0,
    0.1,
    @moonphys.vec3(3.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.axis_angle_basis3(
      @moonphys.vec3(0.0, 0.0, 1.0),
      3.141592653589793 / 2.0,
    ),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(root_state.linear),
      @moonphys.rigid_body_world_body(upper_state.linear),
    ],
    motion_bodies=[
      @moonphys.rigid_body_joint_frame_body(
        root_state,
        @moonphys.rigid_body_inertia("root", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
      @moonphys.rigid_body_joint_frame_body(
        upper_state,
        @moonphys.rigid_body_inertia("upper", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
    ],
    hinge_constraints=[
      @moonphys.rigid_body_hinge_joint_constraint(
        "shoulder",
        "root",
        "upper",
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
        local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
      ),
    ],
  )
  let resolution = @moonphys.rigid_body_world_resolve_hinge_constraints(world)
  assert_eq(resolution.status, "world-hinge-constraint-resolved")
  assert_eq(resolution.hinge_constraint_count, 1)
  assert_eq(resolution.resolved_hinge_constraint_count, 1)
  assert_true(nearly_equal(resolution.max_position_error_m, 1.0))
  assert_true(
    nearly_equal(resolution.max_angular_error_rad, 3.141592653589793 / 2.0),
  )
  assert_true(nearly_equal(resolution.after.bodies[0].state.position.x, 0.5))
  assert_true(nearly_equal(resolution.after.bodies[1].state.position.x, 2.5))
  let root_x = resolution.after.motion_bodies[0].state.angular.orientation.mul_vec3(
    @moonphys.vec3(1.0, 0.0, 0.0),
  )
  assert_true(nearly_equal(root_x.x, @math.cos(3.141592653589793 / 4.0)))
  assert_true(nearly_equal(root_x.y, @math.sin(3.141592653589793 / 4.0)))
  assert_eq(resolution.resolutions[0].status, "world-hinge-resolved")
}

///|
test "rigid body heightfield world step applies hinge constraints" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let root_state = @moonphys.rigid_body_motion_state(
    "root",
    2.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let upper_state = @moonphys.rigid_body_motion_state(
    "upper",
    2.0,
    0.1,
    @moonphys.vec3(3.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.axis_angle_basis3(
      @moonphys.vec3(0.0, 0.0, 1.0),
      3.141592653589793 / 2.0,
    ),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(root_state.linear),
      @moonphys.rigid_body_world_body(upper_state.linear),
    ],
    motion_bodies=[
      @moonphys.rigid_body_joint_frame_body(
        root_state,
        @moonphys.rigid_body_inertia("root", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
      @moonphys.rigid_body_joint_frame_body(
        upper_state,
        @moonphys.rigid_body_inertia("upper", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
    ],
    hinge_constraints=[
      @moonphys.rigid_body_hinge_joint_constraint(
        "shoulder",
        "root",
        "upper",
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
        local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
      ),
    ],
  )
  let step = @moonphys.rigid_body_heightfield_world_step(world, field, 0.0)
  assert_eq(step.status, "world-step-hinge-constrained")
  assert_eq(step.hinge_constraint_count, 1)
  assert_eq(step.resolved_hinge_constraint_count, 1)
  assert_eq(
    step.hinge_constraint_resolution.status,
    "world-hinge-constraint-resolved",
  )
  assert_true(nearly_equal(step.after.bodies[0].state.position.x, 0.5))
  assert_true(nearly_equal(step.after.bodies[1].state.position.x, 2.5))
  assert_true(
    nearly_equal(
      step.after.motion_bodies[0].state.angular.orientation.mul_vec3(
        @moonphys.vec3(1.0, 0.0, 0.0),
      ).y,
      @math.sin(3.141592653589793 / 4.0),
    ),
  )
}

///|
test "rigid body world hinge motors drive matching joints" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let root_state = @moonphys.rigid_body_motion_state(
    "root",
    2.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let upper_state = @moonphys.rigid_body_motion_state(
    "upper",
    2.0,
    0.1,
    @moonphys.vec3(2.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(root_state.linear),
      @moonphys.rigid_body_world_body(upper_state.linear),
    ],
    motion_bodies=[
      @moonphys.rigid_body_joint_frame_body(
        root_state,
        @moonphys.rigid_body_inertia("root", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
      @moonphys.rigid_body_joint_frame_body(
        upper_state,
        @moonphys.rigid_body_inertia("upper", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
    ],
    hinge_constraints=[
      @moonphys.rigid_body_hinge_joint_constraint(
        "shoulder",
        "root",
        "upper",
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
        local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
      ),
    ],
  )
  let motor_frame = @moonphys.integrate_joint_motor_frame(
    "hinge-motor-frame",
    [@moonphys.joint_motor_state("shoulder", 0.0, 0.0)],
    [@moonphys.joint_motor_command("shoulder", 1.0, 0.0, 0.0)],
    [@moonphys.joint_servo_config("shoulder", 2.0, 0.0)],
    [@moonphys.joint_limit("shoulder", -2.0, 2.0, 4.0, 10.0)],
    [2.0],
    0.5,
  )
  let driven = @moonphys.rigid_body_world_integrate_hinge_motors(
    world, motor_frame,
  )
  assert_eq(driven.status, "world-hinge-motor-driven")
  assert_eq(driven.driven_joint_count, 1)
  assert_eq(driven.review_count, 0)
  assert_true(nearly_equal(driven.max_abs_angle_delta_rad, 0.25))
  assert_eq(driven.steps[0].status, "world-hinge-motor-driven")
  assert_eq(
    driven.hinge_constraint_resolution.status,
    "world-hinge-constraint-resolved",
  )
  assert_eq(
    driven.hinge_constraint_resolution.resolved_hinge_constraint_count,
    1,
  )
  assert_true(driven.hinge_constraint_resolution.angular_impulse_nms > 0.0)
  let upper_x = driven.after.motion_bodies[1].state.angular.orientation.mul_vec3(
    @moonphys.vec3(1.0, 0.0, 0.0),
  )
  assert_true(nearly_equal(upper_x.x, @math.cos(0.25)))
  assert_true(nearly_equal(upper_x.z, 0.0 - @math.sin(0.25)))
}

///|
test "rigid body world hinge motor trace replays sequential frames" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let root_state = @moonphys.rigid_body_motion_state(
    "root",
    2.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let upper_state = @moonphys.rigid_body_motion_state(
    "upper",
    2.0,
    0.1,
    @moonphys.vec3(2.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(root_state.linear),
      @moonphys.rigid_body_world_body(upper_state.linear),
    ],
    motion_bodies=[
      @moonphys.rigid_body_joint_frame_body(
        root_state,
        @moonphys.rigid_body_inertia("root", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
      @moonphys.rigid_body_joint_frame_body(
        upper_state,
        @moonphys.rigid_body_inertia("upper", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
    ],
    hinge_constraints=[
      @moonphys.rigid_body_hinge_joint_constraint(
        "shoulder",
        "root",
        "upper",
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
        local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
      ),
    ],
  )
  let frame0 = @moonphys.integrate_joint_motor_frame(
    "hinge-motor-frame/0",
    [@moonphys.joint_motor_state("shoulder", 0.0, 0.0)],
    [@moonphys.joint_motor_command("shoulder", 1.0, 0.0, 0.0)],
    [@moonphys.joint_servo_config("shoulder", 2.0, 0.0)],
    [@moonphys.joint_limit("shoulder", -2.0, 2.0, 4.0, 10.0)],
    [2.0],
    0.5,
  )
  let frame1 = @moonphys.integrate_joint_motor_frame(
    "hinge-motor-frame/1",
    [@moonphys.joint_motor_state("shoulder", 0.25, 0.0)],
    [@moonphys.joint_motor_command("shoulder", 1.0, 0.0, 0.0)],
    [@moonphys.joint_servo_config("shoulder", 2.0, 0.0)],
    [@moonphys.joint_limit("shoulder", -2.0, 2.0, 4.0, 10.0)],
    [2.0],
    0.5,
  )
  let trace = @moonphys.rigid_body_world_integrate_hinge_motor_trace(
    "hinge-motor-trace",
    world,
    [frame0, frame1],
    note="sequential hinge motor replay",
  )
  assert_eq(trace.status, "world-hinge-motor-trace-driven")
  assert_eq(trace.frame_count, 2)
  assert_eq(trace.driven_joint_count, 2)
  assert_eq(trace.review_count, 0)
  assert_true(trace.max_abs_angle_delta_rad > 0.0)
  assert_true(trace.max_abs_velocity_delta_rad_s >= 0.0)
  assert_true(trace.final_world == trace.frames[1].after)
  assert_true(
    trace.frames[1].before.motion_bodies[1].state.angular.orientation ==
    trace.frames[0].after.motion_bodies[1].state.angular.orientation,
  )
}

///|
test "rigid body heightfield world hinge motor trace couples drive and contact" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let root_state = @moonphys.rigid_body_motion_state(
    "root",
    2.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 0.08),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let upper_state = @moonphys.rigid_body_motion_state(
    "upper",
    2.0,
    0.1,
    @moonphys.vec3(2.0, 0.0, 0.08),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(root_state.linear),
      @moonphys.rigid_body_world_body(upper_state.linear),
    ],
    motion_bodies=[
      @moonphys.rigid_body_joint_frame_body(
        root_state,
        @moonphys.rigid_body_inertia("root", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
      @moonphys.rigid_body_joint_frame_body(
        upper_state,
        @moonphys.rigid_body_inertia("upper", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
    ],
    hinge_constraints=[
      @moonphys.rigid_body_hinge_joint_constraint(
        "shoulder",
        "root",
        "upper",
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
        local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
      ),
    ],
  )
  let frame0 = @moonphys.integrate_joint_motor_frame(
    "hinge-motor-frame/0",
    [@moonphys.joint_motor_state("shoulder", 0.0, 0.0)],
    [@moonphys.joint_motor_command("shoulder", 1.0, 0.0, 0.0)],
    [@moonphys.joint_servo_config("shoulder", 2.0, 0.0)],
    [@moonphys.joint_limit("shoulder", -2.0, 2.0, 4.0, 10.0)],
    [2.0],
    0.5,
  )
  let frame1 = @moonphys.integrate_joint_motor_frame(
    "hinge-motor-frame/1",
    [@moonphys.joint_motor_state("shoulder", 0.25, 0.0)],
    [@moonphys.joint_motor_command("shoulder", 1.0, 0.0, 0.0)],
    [@moonphys.joint_servo_config("shoulder", 2.0, 0.0)],
    [@moonphys.joint_limit("shoulder", -2.0, 2.0, 4.0, 10.0)],
    [2.0],
    0.5,
  )
  let trace = @moonphys.rigid_body_heightfield_world_hinge_motor_trace(
    "hinge-motor-heightfield-trace",
    world,
    field,
    [frame0, frame1],
    note="motor-driven world replay",
  )
  assert_eq(trace.status, "world-heightfield-hinge-motor-trace-driven")
  assert_eq(trace.frame_count, 2)
  assert_eq(trace.motor_frame_count, 2)
  assert_eq(trace.driven_joint_count, 2)
  assert_eq(trace.contact_count, 4)
  assert_true(trace.resolved_hinge_constraint_count >= 2)
  assert_eq(trace.review_count, 0)
  assert_eq(trace.frames[0].status, "world-heightfield-hinge-motor-contact")
  assert_true(trace.frames[0].contact_count == 2)
  assert_true(trace.frames[0].world_step.contact_count == 2)
  assert_true(trace.frames[0].motor_drive.driven_joint_count == 1)
  assert_true(trace.final_world == trace.frames[1].after)
  let envelope = @moonphys.rigid_body_heightfield_world_hinge_motor_trace_envelope(
    trace,
  )
  assert_eq(envelope.status, "world-trace-envelope-bounded")
  assert_eq(envelope.frame_count, 2)
  assert_eq(envelope.body_count, 2)
  assert_eq(envelope.body_sample_count, 6)
  assert_eq(envelope.contact_count, trace.contact_count)
  assert_eq(
    envelope.resolved_hinge_constraint_count,
    trace.resolved_hinge_constraint_count,
  )
  assert_true(envelope.max_speed_mps >= 0.0)
  assert_true(envelope.max_kinetic_energy_j >= 0.0)
  assert_true(envelope.max_body_linear_momentum_kg_mps >= 0.0)
  assert_true(envelope.max_world_linear_momentum_kg_mps >= 0.0)
  assert_true(envelope.max_frame_kinetic_energy_delta_j >= 0.0)
  assert_true(envelope.min_position.z >= 0.0)
  assert_true(envelope.max_position.x >= 2.0)
  let support_trace = @moonphys.rigid_body_heightfield_world_hinge_motor_support_trace(
    trace,
  )
  assert_eq(support_trace.trace_id, trace.trace_id)
  assert_eq(support_trace.frame_count, trace.frame_count)
  assert_eq(support_trace.body_count, 2)
  assert_eq(support_trace.contact_count, trace.contact_count)
  assert_true(support_trace.support_frame_count == 2)
  assert_true(support_trace.stable_frame_count >= 0)
  assert_true(support_trace.min_planar_margin_m <= 1.0)
  assert_true(support_trace.max_support_recovery_shift_m >= 0.0)
  assert_true(support_trace.max_center_of_pressure_error_m >= 0.0)
  assert_true(support_trace.status.contains("world-support-trace"))
  let dynamic_support_trace = @moonphys.rigid_body_heightfield_world_hinge_motor_dynamic_support_trace(
    trace,
  )
  assert_eq(dynamic_support_trace.trace_id, trace.trace_id)
  assert_eq(dynamic_support_trace.frame_count, trace.frame_count)
  assert_eq(dynamic_support_trace.body_count, 2)
  assert_eq(dynamic_support_trace.contact_count, trace.contact_count)
  assert_eq(
    dynamic_support_trace.capture_stable_frame_count +
    dynamic_support_trace.review_frame_count,
    dynamic_support_trace.frame_count,
  )
  assert_true(dynamic_support_trace.worst_capture_margin_m <= 1.0)
  assert_true(dynamic_support_trace.max_capture_recovery_shift_m >= 0.0)
  assert_true(
    dynamic_support_trace.status.contains("world-dynamic-support-trace"),
  )
  let review = @moonphys.rigid_body_heightfield_world_hinge_motor_replay_review(
    trace,
  )
  assert_eq(review.trace_id, trace.trace_id)
  assert_eq(review.frame_count, trace.frame_count)
  assert_eq(review.body_count, 2)
  assert_eq(review.body_sample_count, envelope.body_sample_count)
  assert_eq(review.support_contact_count, support_trace.contact_count)
  assert_eq(
    review.capture_review_frame_count,
    dynamic_support_trace.review_frame_count,
  )
  assert_true(review.max_position.x >= envelope.max_position.x)
  assert_true(review.max_world_linear_momentum_kg_mps >= 0.0)
  assert_eq(
    review.max_support_recovery_shift_m,
    support_trace.max_support_recovery_shift_m,
  )
  assert_eq(
    review.max_capture_recovery_shift_m,
    dynamic_support_trace.max_capture_recovery_shift_m,
  )
  assert_eq(review.ready, review.blocker_count == 0)
  assert_eq(review.blocker_count, review.blockers.length())
  assert_true(review.status.contains("world-replay-review"))
}

///|
test "hinge motor replay review can use explicit support footprints" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let body = @moonphys.rigid_body_state(
    "body",
    1.0,
    0.05,
    @moonphys.vec3(0.0, 0.0, 0.1),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "support-target-world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [@moonphys.rigid_body_world_body(body)],
  )
  let motor = @moonphys.integrate_joint_motor_frame(
    "idle",
    [],
    [],
    [],
    [],
    [],
    0.1,
  )
  let trace = @moonphys.rigid_body_heightfield_world_hinge_motor_trace(
    "support-target-trace",
    world,
    field,
    [motor],
  )
  let footprints = [
    [
      @moonphys.support_footprint(
        "left-foot",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
  ]
  let support_trace = @moonphys.rigid_body_heightfield_world_hinge_motor_support_trace_with_footprints(
    trace, footprints,
  )
  assert_eq(support_trace.contact_count, 1)
  assert_eq(support_trace.review_frame_count, 0)
  assert_eq(support_trace.status, "world-support-trace-stable")
  let review = @moonphys.rigid_body_heightfield_world_hinge_motor_replay_review_with_footprints(
    trace, footprints,
  )
  assert_true(!review.blockers.contains("world-support-review"))
  assert_eq(review.support_contact_count, 1)
  assert_eq(review.support_review_frame_count, 0)
}

///|
test "hinge motor replay review can use center-of-mass targets" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let body = @moonphys.rigid_body_state(
    "body",
    1.0,
    0.05,
    @moonphys.vec3(1.0, 0.0, 0.1),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "support-com-target-world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [@moonphys.rigid_body_world_body(body)],
  )
  let motor = @moonphys.integrate_joint_motor_frame(
    "idle",
    [],
    [],
    [],
    [],
    [],
    0.1,
  )
  let trace = @moonphys.rigid_body_heightfield_world_hinge_motor_trace(
    "support-com-target-trace",
    world,
    field,
    [motor],
  )
  let footprints = [
    [
      @moonphys.support_footprint(
        "left-foot",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
  ]
  let footprint_review = @moonphys.rigid_body_heightfield_world_hinge_motor_replay_review_with_footprints(
    trace, footprints,
  )
  assert_true(footprint_review.blockers.contains("world-support-review"))
  let targets = [
    @moonphys.rigid_body_world_center_of_mass_target(
      @moonphys.vec3(0.0, 0.0, 0.1),
      @moonphys.vec3(0.0, 0.0, 0.0),
      total_mass_kg=1.0,
      nominal_com_height_m=0.72,
    ),
  ]
  let targeted_review = @moonphys.rigid_body_heightfield_world_hinge_motor_replay_review_with_targets(
    trace, footprints, targets,
  )
  assert_true(!targeted_review.blockers.contains("world-support-review"))
  assert_true(
    !targeted_review.blockers.contains("world-dynamic-support-review"),
  )
  assert_eq(targeted_review.support_review_frame_count, 0)
  assert_eq(targeted_review.capture_review_frame_count, 0)
}

///|
test "replay review accepts walking frames when dynamic support is clear" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let body = @moonphys.rigid_body_state(
    "body",
    1.0,
    0.05,
    @moonphys.vec3(1.0, 0.0, 0.1),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "dynamic-walk-world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [@moonphys.rigid_body_world_body(body)],
  )
  let motor = @moonphys.integrate_joint_motor_frame(
    "idle",
    [],
    [],
    [],
    [],
    [],
    0.1,
  )
  let trace = @moonphys.rigid_body_heightfield_world_hinge_motor_trace(
    "dynamic-walk-trace",
    world,
    field,
    [motor],
  )
  let footprints = [
    [
      @moonphys.support_footprint(
        "left-foot",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
  ]
  let targets = [
    @moonphys.rigid_body_world_center_of_mass_target(
      @moonphys.vec3(0.0, 0.0, 0.1),
      @moonphys.vec3(0.0, 0.0, 0.0),
      total_mass_kg=1.0,
      nominal_com_height_m=0.72,
    ),
  ]
  let envelope = @moonphys.rigid_body_heightfield_world_hinge_motor_trace_envelope(
    trace,
  )
  let static_support = @moonphys.rigid_body_heightfield_world_hinge_motor_support_trace_with_footprints(
    trace, footprints,
  )
  let dynamic_support = @moonphys.rigid_body_heightfield_world_hinge_motor_dynamic_support_trace_with_targets(
    trace, footprints, targets,
  )
  let review = @moonphys.rigid_body_world_replay_review(
    envelope, static_support, dynamic_support,
  )
  assert_true(static_support.review_frame_count > 0)
  assert_eq(dynamic_support.review_frame_count, 0)
  assert_true(review.ready)
  assert_eq(review.status, "world-replay-review-ready")
  assert_true(!review.blockers.contains("world-support-review"))
}
