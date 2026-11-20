///|
test "rigid body world body targets update body and motion state" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.2,
    @moonphys.vec3(0.0, 0.0, 1.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let motion = @moonphys.rigid_body_joint_frame_body(
    @moonphys.rigid_body_motion_state(
      "body",
      4.0,
      0.2,
      state.position,
      state.velocity,
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.sphere_diagonal_inertia("body", 4.0, 0.2),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "target-world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [@moonphys.rigid_body_world_body(state)],
    motion_bodies=[motion],
  )
  let target = @moonphys.rigid_body_world_body_target(
    "body",
    @moonphys.vec3(1.0, 0.0, 1.0),
    @moonphys.vec3(2.0, 0.0, 0.0),
    max_correction_m=2.0,
  )
  let frame = @moonphys.rigid_body_world_apply_body_targets(world, [target])
  assert_eq(frame.status, "body-target-frame-applied")
  assert_eq(frame.target_count, 1)
  assert_eq(frame.applied_count, 1)
  assert_eq(frame.review_count, 0)
  assert_true(nearly_equal(frame.max_correction_m, 1.0))
  assert_true(frame.after.bodies[0].state.position == target.target_position)
  assert_true(frame.after.bodies[0].state.velocity == target.target_velocity)
  assert_true(
    frame.after.motion_bodies[0].state.linear == frame.after.bodies[0].state,
  )
}

///|
test "rigid body world body targets review missing body" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "target-world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [],
  )
  let frame = @moonphys.rigid_body_world_apply_body_targets(world, [
    @moonphys.rigid_body_world_body_target(
      "missing",
      @moonphys.vec3(0.0, 0.0, 1.0),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
  ])
  assert_eq(frame.status, "body-target-frame-review")
  assert_eq(frame.applied_count, 0)
  assert_eq(frame.review_count, 1)
  assert_eq(frame.steps[0].status, "body-target-missing-body-review")
}

///|
test "rigid body heightfield world trace steps multiple bodies" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "falling",
          4.0,
          0.2,
          @moonphys.vec3(0.0, 0.0, 0.25),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "driven",
          2.0,
          0.1,
          @moonphys.vec3(1.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
        external_force_n=@moonphys.vec3(4.0, 0.0, 0.0),
      ),
    ],
  )
  let step = @moonphys.rigid_body_heightfield_world_step(world, field, 0.2)
  assert_eq(step.status, "world-step-contact")
  assert_eq(step.body_count, 2)
  assert_eq(step.contact_count, 1)
  assert_eq(step.body_pair_count, 1)
  assert_eq(step.body_pair_contact_count, 0)
  assert_eq(step.review_count, 0)
  assert_true(step.body_frames[0].status == "world-body-contact-resolved")
  assert_true(step.body_frames[1].status == "world-body-clear")
  assert_true(step.body_frames[1].acceleration_mps2.x == 2.0)
  assert_true(step.body_frames[1].after.position.x > 1.0)
  assert_true(step.kinetic_energy_after_j > step.kinetic_energy_before_j)
  assert_true(step.after.bodies[0].state == step.body_frames[0].after)
}

///|
test "rigid body world contact schedule skips broadphase clear pairs" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "left",
          1.0,
          0.25,
          @moonphys.vec3(0.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "right",
          1.0,
          0.25,
          @moonphys.vec3(2.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
    ],
  )
  let schedule = @moonphys.rigid_body_world_contact_schedule(world)
  assert_eq(schedule.status, "world-contact-clear")
  assert_eq(schedule.pair_count, 1)
  assert_eq(schedule.broadphase_overlap_count, 0)
  assert_eq(schedule.contact_count, 0)
  assert_eq(schedule.pairs[0].status, "world-pair-broadphase-clear")
  assert_eq(schedule.pairs[0].contact.status, "broadphase-clear")
  assert_true(nearly_equal(schedule.pairs[0].contact.normal.length(), 1.0))
}

///|
test "rigid body world contact schedule records body contacts" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "box-a",
          1.0,
          0.0,
          @moonphys.vec3(0.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
        collision_shape=@moonphys.collision_box(
          "box-a-shape",
          @moonphys.vec3(0.5, 0.5, 0.5),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "box-b",
          1.0,
          0.0,
          @moonphys.vec3(0.9, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
        collision_shape=@moonphys.collision_box(
          "box-b-shape",
          @moonphys.vec3(0.5, 0.5, 0.5),
        ),
      ),
    ],
  )
  let schedule = @moonphys.rigid_body_world_contact_schedule(world)
  assert_eq(schedule.status, "world-contact-scheduled")
  assert_eq(schedule.pair_count, 1)
  assert_eq(schedule.broadphase_overlap_count, 1)
  assert_eq(schedule.contact_count, 1)
  assert_eq(schedule.manifold.status, "single-contact")
  assert_eq(schedule.pairs[0].status, "world-pair-contact")
  assert_eq(schedule.pairs[0].contact.status, "box-box-contact")
  let step = @moonphys.rigid_body_heightfield_world_step(world, field, 0.1)
  assert_eq(step.status, "world-step-body-contact")
  assert_eq(step.contact_count, 0)
  assert_eq(step.body_pair_contact_count, 1)
  assert_eq(step.body_contact_schedule.manifold.status, "single-contact")
}

///|
test "rigid body world contact resolution separates overlapping bodies" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.material_contact_model("test-material", 0.0, 0.0),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "sphere-a",
          1.0,
          0.5,
          @moonphys.vec3(0.0, 0.0, 2.0),
          @moonphys.vec3(1.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "sphere-b",
          1.0,
          0.5,
          @moonphys.vec3(0.9, 0.0, 2.0),
          @moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        ),
      ),
    ],
  )
  let schedule = @moonphys.rigid_body_world_contact_schedule(world)
  let resolution = @moonphys.rigid_body_world_resolve_contacts(world, schedule)
  assert_eq(resolution.status, "world-contact-resolved")
  assert_eq(resolution.resolved_contact_count, 1)
  assert_true(nearly_equal(resolution.normal_impulse_ns, 1.0))
  assert_true(resolution.resolutions[0].status == "world-pair-resolved")
  assert_true(nearly_equal(resolution.after.bodies[0].state.position.x, -0.05))
  assert_true(nearly_equal(resolution.after.bodies[1].state.position.x, 0.95))
  assert_true(nearly_equal(resolution.after.bodies[0].state.velocity.x, 0.0))
  assert_true(nearly_equal(resolution.after.bodies[1].state.velocity.x, 0.0))
}

///|
test "rigid body heightfield world step applies body contact response" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.material_contact_model("test-material", 0.0, 0.0),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "sphere-a",
          1.0,
          0.5,
          @moonphys.vec3(0.0, 0.0, 2.0),
          @moonphys.vec3(1.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "sphere-b",
          1.0,
          0.5,
          @moonphys.vec3(0.9, 0.0, 2.0),
          @moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        ),
      ),
    ],
  )
  let step = @moonphys.rigid_body_heightfield_world_step(world, field, 0.0)
  assert_eq(step.status, "world-step-body-contact")
  assert_eq(step.body_contact_resolution.status, "world-contact-resolved")
  assert_eq(step.body_contact_resolution.resolved_contact_count, 1)
  assert_true(step.kinetic_energy_after_j < step.kinetic_energy_before_j)
  assert_true(nearly_equal(step.after.bodies[0].state.position.x, -0.05))
  assert_true(nearly_equal(step.after.bodies[1].state.position.x, 0.95))
  assert_true(nearly_equal(step.after.bodies[0].state.velocity.x, 0.0))
  assert_true(nearly_equal(step.after.bodies[1].state.velocity.x, 0.0))
}

///|
test "rigid body world constraints restore distance" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "left",
          1.0,
          0.1,
          @moonphys.vec3(0.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "right",
          3.0,
          0.1,
          @moonphys.vec3(2.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
    ],
    constraints=[
      @moonphys.rigid_body_world_distance_constraint(
        "link", "left", "right", 1.0,
      ),
    ],
  )
  let resolution = @moonphys.rigid_body_world_resolve_constraints(world)
  assert_eq(resolution.status, "world-constraint-resolved")
  assert_eq(resolution.constraint_count, 1)
  assert_eq(resolution.resolved_constraint_count, 1)
  assert_true(nearly_equal(resolution.max_error_m, 1.0))
  assert_true(nearly_equal(resolution.after.bodies[0].state.position.x, 0.75))
  assert_true(nearly_equal(resolution.after.bodies[1].state.position.x, 1.75))
  assert_true(nearly_equal(resolution.resolutions[0].correction_a.x, 0.75))
  assert_true(nearly_equal(resolution.resolutions[0].correction_b.x, -0.25))
}

///|
test "rigid body world constraints resolve anchored points" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "upper-link",
          1.0,
          0.1,
          @moonphys.vec3(0.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "lower-link",
          1.0,
          0.1,
          @moonphys.vec3(3.0, 0.0, 2.0),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
    ],
    constraints=[
      @moonphys.rigid_body_world_distance_constraint(
        "knee-pin",
        "upper-link",
        "lower-link",
        0.0,
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
      ),
    ],
  )
  let resolution = @moonphys.rigid_body_world_resolve_constraints(world)
  let resolved = resolution.resolutions[0]
  assert_eq(resolution.status, "world-constraint-resolved")
  assert_eq(resolved.status, "world-constraint-resolved")
  assert_true(nearly_equal(resolved.current_length_m, 1.0))
  assert_true(nearly_equal(resolved.world_anchor_a.x, 1.0))
  assert_true(nearly_equal(resolved.world_anchor_b.x, 2.0))
  assert_true(nearly_equal(resolution.after.bodies[0].state.position.x, 0.5))
  assert_true(nearly_equal(resolution.after.bodies[1].state.position.x, 2.5))
  assert_true(
    nearly_equal(
      resolution.after.bodies[0].state.position.add(resolved.anchor_a_m).x,
      resolution.after.bodies[1].state.position.add(resolved.anchor_b_m).x,
    ),
  )
}

///|
test "rigid body heightfield world step applies constraints" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "left",
          1.0,
          0.1,
          @moonphys.vec3(0.0, 0.0, 2.0),
          @moonphys.vec3(1.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "right",
          1.0,
          0.1,
          @moonphys.vec3(2.0, 0.0, 2.0),
          @moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
        ),
      ),
    ],
    constraints=[
      @moonphys.rigid_body_world_distance_constraint(
        "link", "left", "right", 1.0,
      ),
    ],
  )
  let step = @moonphys.rigid_body_heightfield_world_step(world, field, 0.0)
  assert_eq(step.status, "world-step-constrained")
  assert_eq(step.constraint_count, 1)
  assert_eq(step.resolved_constraint_count, 1)
  assert_eq(step.constraint_resolution.status, "world-constraint-resolved")
  assert_true(nearly_equal(step.after.bodies[0].state.position.x, 0.5))
  assert_true(nearly_equal(step.after.bodies[1].state.position.x, 1.5))
  assert_true(nearly_equal(step.after.bodies[0].state.velocity.x, 0.0))
  assert_true(nearly_equal(step.after.bodies[1].state.velocity.x, 0.0))
}

///|
test "rigid body heightfield world trace is deterministic" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "body",
          4.0,
          0.2,
          @moonphys.vec3(0.0, 0.0, 0.25),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
    ],
  )
  let trace = @moonphys.rigid_body_heightfield_world_trace(
    "world-trace", world, field, 0.2, 4, "test world trace",
  )
  let replay = @moonphys.rigid_body_heightfield_world_trace(
    "world-trace", world, field, 0.2, 4, "test world trace",
  )
  assert_true(trace == replay)
  assert_eq(trace.status, "world-deterministic-replay")
  assert_eq(trace.frame_count, 4)
  assert_true(trace.frames[0].contact_count == 1)
  assert_true(
    trace.final_world.bodies[0].state == trace.frames[3].after.bodies[0].state,
  )
  let envelope = @moonphys.rigid_body_heightfield_world_trace_envelope(trace)
  assert_eq(envelope.status, "world-trace-envelope-bounded")
  assert_eq(envelope.frame_count, 4)
  assert_eq(envelope.body_count, 1)
  assert_eq(envelope.body_sample_count, 5)
  assert_true(envelope.contact_count >= trace.frames[0].contact_count)
  assert_true(envelope.max_speed_mps >= 0.0)
  assert_true(envelope.max_kinetic_energy_j >= 0.0)
  assert_true(nearly_equal(envelope.total_mass_kg, 4.0))
  assert_true(envelope.initial_center_of_mass == @moonphys.vec3(0.0, 0.0, 0.25))
  assert_true(envelope.max_center_of_mass_speed_mps >= 0.0)
  assert_true(envelope.max_body_linear_momentum_kg_mps >= 0.0)
  assert_true(envelope.max_world_linear_momentum_kg_mps >= 0.0)
  assert_true(envelope.max_frame_kinetic_energy_delta_j >= 0.0)
  let support_trace = @moonphys.rigid_body_heightfield_world_support_trace(
    trace,
  )
  assert_eq(support_trace.frame_count, 4)
  assert_eq(support_trace.body_count, 1)
  assert_true(support_trace.contact_count >= 1)
  assert_true(support_trace.min_planar_margin_m >= 0.0)
  assert_true(support_trace.status.contains("world-support-trace"))
  let dynamic_support_trace = @moonphys.rigid_body_heightfield_world_dynamic_support_trace(
    trace,
  )
  assert_eq(dynamic_support_trace.frame_count, 4)
  assert_eq(dynamic_support_trace.body_count, 1)
  assert_eq(
    dynamic_support_trace.capture_stable_frame_count +
    dynamic_support_trace.review_frame_count,
    dynamic_support_trace.frame_count,
  )
  assert_true(
    dynamic_support_trace.status.contains("world-dynamic-support-trace"),
  )
  let review = @moonphys.rigid_body_heightfield_world_replay_review(trace)
  assert_eq(review.frame_count, 4)
  assert_eq(review.body_count, 1)
  assert_eq(review.support_contact_count, support_trace.contact_count)
  assert_eq(review.blocker_count, review.blockers.length())
  assert_true(review.status.contains("world-replay-review"))
}

///|
test "rigid body world center of mass is mass weighted" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "light",
          2.0,
          0.2,
          @moonphys.vec3(0.0, 0.0, 1.0),
          @moonphys.vec3(1.0, 0.0, 0.0),
        ),
      ),
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "heavy",
          4.0,
          0.2,
          @moonphys.vec3(3.0, 0.0, 1.0),
          @moonphys.vec3(0.0, 3.0, 0.0),
        ),
      ),
    ],
  )
  assert_true(nearly_equal(@moonphys.rigid_body_world_total_mass(world), 6.0))
  assert_true(
    @moonphys.rigid_body_world_center_of_mass(world) ==
    @moonphys.vec3(2.0, 0.0, 1.0),
  )
  let center_velocity = @moonphys.rigid_body_world_center_of_mass_velocity(
    world,
  )
  assert_true(nearly_equal(center_velocity.x, 1.0 / 3.0))
  assert_true(nearly_equal(center_velocity.y, 2.0))
  assert_true(nearly_equal(center_velocity.z, 0.0))
}

///|
test "rigid body linear momentum follows mass and velocity" {
  let state = @moonphys.rigid_body_state(
    "body",
    3.0,
    0.2,
    @moonphys.vec3(0.0, 0.0, 1.0),
    @moonphys.vec3(2.0, -1.0, 0.5),
  )
  assert_true(
    @moonphys.rigid_body_linear_momentum(state) ==
    @moonphys.vec3(6.0, -3.0, 1.5),
  )
}

///|
test "rigid body heightfield world step flags review states" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let other_field = @moonphys.heightfield("other", 1, 1, 1.0, [0.0])
  let world = @moonphys.rigid_body_heightfield_world(
    "world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [
      @moonphys.rigid_body_world_body(
        @moonphys.rigid_body_state(
          "massless",
          0.0,
          0.2,
          @moonphys.vec3(0.0, 0.0, 0.25),
          @moonphys.vec3(0.0, 0.0, 0.0),
        ),
      ),
    ],
  )
  let reviewed = @moonphys.rigid_body_heightfield_world_step(world, field, 0.2)
  assert_eq(reviewed.status, "world-step-review")
  assert_eq(reviewed.review_count, 1)
  assert_eq(reviewed.body_frames[0].status, "world-body-zero-mass-review")
  let mismatched = @moonphys.rigid_body_heightfield_world_step(
    world, other_field, 0.2,
  )
  assert_eq(mismatched.status, "world-heightfield-mismatch-review")
}
