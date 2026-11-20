///|
test "motion frame review accepts stable contact evidence" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "left-sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  let contact = @moonphys.motion_contact_evidence(
    "left-contact",
    @moonphys.support_footprint("left", @moonphys.vec3(0.0, 0.0, 0.0), 0.2, 0.1),
    patch,
    @moonphys.vec3(0.0, 0.0, 120.0),
  )
  let review = @moonphys.motion_frame_review(
    "frame-0",
    @moonphys.lunar_environment(),
    @moonphys.material_contact_model("test", 0.8, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.0, 0.0, 0.0),
    [contact],
  )
  assert_true(review.ready)
  assert_eq(review.status, "motion-frame-review-ready")
  assert_eq(review.blocker_count, 0)
  assert_eq(review.contact_count, 1)
  assert_eq(review.active_footprint_count, 1)
  assert_eq(review.support_status, "inside-support")
  assert_eq(review.support_load_status, "single-support-load")
  assert_eq(review.capture_status, "capture-point-inside-support")
  assert_eq(review.contact_reviews[0].traction_status, "traction-ok")
  assert_true(nearly_equal(review.total_normal_force_n, 120.0))
  assert_true(nearly_equal(review.estimated_mass_kg, 120.0 / 1.625))
  assert_true(nearly_equal(review.center_of_mass_speed_mps, 0.0))
  assert_true(nearly_equal(review.linear_momentum_kg_mps, 0.0))
  assert_true(nearly_equal(review.linear_kinetic_energy_j, 0.0))
}

///|
test "motion frame review blocks traction and capture failures" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "left-sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  let contact = @moonphys.motion_contact_evidence(
    "left-contact",
    @moonphys.support_footprint("left", @moonphys.vec3(0.0, 0.0, 0.0), 0.2, 0.1),
    patch,
    @moonphys.vec3(100.0, 0.0, 20.0),
  )
  let review = @moonphys.motion_frame_review(
    "frame-slip",
    @moonphys.lunar_environment(),
    @moonphys.material_contact_model("low-friction", 0.25, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.8, 0.0, 0.0),
    [contact],
  )
  assert_true(!review.ready)
  assert_eq(review.status, "motion-frame-review-blocked")
  assert_true(review.blocker_count >= 2)
  assert_eq(review.traction_review_count, 1)
  assert_eq(
    review.contact_reviews[0].traction_status,
    "friction-limit-exceeded",
  )
  assert_eq(review.capture_status, "capture-point-outside-support")
  assert_true(review.blockers.contains("capture:capture-point-outside-support"))
  assert_true(
    review.blockers.contains("contact:left-contact:motion-contact-review"),
  )
}

///|
test "motion frame trace review envelopes contact force and torque" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let material = @moonphys.material_contact_model("test", 0.8, 0.0)
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "left-sole",
    @moonphys.vec3(0.1, 0.0, 0.0),
    0.2,
    0.1,
  )
  let frame_0 = @moonphys.motion_frame_review(
    "frame-0",
    @moonphys.lunar_environment(),
    material,
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.0, 0.0, 0.0),
    [
      @moonphys.motion_contact_evidence(
        "left-contact",
        @moonphys.support_footprint(
          "left",
          @moonphys.vec3(0.1, 0.0, 0.0),
          0.2,
          0.1,
        ),
        patch,
        @moonphys.vec3(0.0, 0.0, 80.0),
      ),
    ],
  )
  let frame_1 = @moonphys.motion_frame_review(
    "frame-1",
    @moonphys.lunar_environment(),
    material,
    @moonphys.vec3(0.05, 0.0, 0.7),
    @moonphys.vec3(0.2, 0.0, 0.0),
    [
      @moonphys.motion_contact_evidence(
        "left-contact",
        @moonphys.support_footprint(
          "left",
          @moonphys.vec3(0.1, 0.0, 0.0),
          0.2,
          0.1,
        ),
        patch,
        @moonphys.vec3(8.0, 0.0, 120.0),
      ),
    ],
  )
  let trace = @moonphys.motion_frame_trace_review("walk-trace", [
    frame_0, frame_1,
  ])
  assert_true(trace.ready)
  assert_eq(trace.status, "motion-frame-trace-review-ready")
  assert_eq(trace.frame_count, 2)
  assert_eq(trace.ready_frame_count, 2)
  assert_eq(trace.blocked_frame_count, 0)
  assert_eq(trace.contact_count, 2)
  assert_eq(trace.contact_review_count, 2)
  assert_true(nearly_equal(trace.max_total_normal_force_n, 120.0))
  assert_true(nearly_equal(trace.max_contact_normal_force_n, 120.0))
  assert_true(nearly_equal(trace.max_contact_tangential_force_n, 8.0))
  assert_true(nearly_equal(trace.max_contact_force_n, 120.0))
  assert_true(trace.max_contact_torque_nm > 0.0)
  assert_true(trace.max_friction_utilization > 0.0)
  assert_true(trace.max_pressure_pa > 0.0)
  assert_true(nearly_equal(trace.max_estimated_mass_kg, 120.0 / 1.625))
  assert_true(nearly_equal(trace.max_center_of_mass_speed_mps, 0.2))
  assert_true(
    nearly_equal(trace.max_linear_momentum_kg_mps, 120.0 / 1.625 * 0.2),
  )
  assert_true(
    nearly_equal(
      trace.max_linear_kinetic_energy_j,
      0.5 * 120.0 / 1.625 * 0.2 * 0.2,
    ),
  )
}

///|
test "motion frame review can allow capture-stable dynamic support" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "left-sole",
    @moonphys.vec3(0.2, 0.0, 0.0),
    0.05,
    0.1,
  )
  let contact = @moonphys.motion_contact_evidence(
    "left-contact",
    @moonphys.support_footprint(
      "left",
      @moonphys.vec3(0.2, 0.0, 0.0),
      0.05,
      0.1,
    ),
    patch,
    @moonphys.vec3(0.0, 0.0, 80.0),
  )
  let static_review = @moonphys.motion_frame_review(
    "static-single-stance",
    @moonphys.lunar_environment(),
    @moonphys.material_contact_model("test", 0.8, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.305, 0.0, 0.0),
    [contact],
  )
  let dynamic_review = @moonphys.motion_frame_review(
    "dynamic-single-stance",
    @moonphys.lunar_environment(),
    @moonphys.material_contact_model("test", 0.8, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.305, 0.0, 0.0),
    [contact],
    allow_dynamic_support=true,
  )
  assert_true(!static_review.ready)
  assert_eq(static_review.support_status, "outside-support")
  assert_true(dynamic_review.ready)
  assert_eq(dynamic_review.support_status, "outside-support")
  assert_eq(dynamic_review.capture_status, "capture-point-inside-support")
}

///|
test "motion frame trace review reports blocked frames" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "left-sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  let blocked = @moonphys.motion_frame_review(
    "frame-slip",
    @moonphys.lunar_environment(),
    @moonphys.material_contact_model("low-friction", 0.25, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.8, 0.0, 0.0),
    [
      @moonphys.motion_contact_evidence(
        "left-contact",
        @moonphys.support_footprint(
          "left",
          @moonphys.vec3(0.0, 0.0, 0.0),
          0.2,
          0.1,
        ),
        patch,
        @moonphys.vec3(100.0, 0.0, 20.0),
      ),
    ],
  )
  let trace = @moonphys.motion_frame_trace_review("blocked-trace", [blocked])
  assert_true(!trace.ready)
  assert_eq(trace.status, "motion-frame-trace-review-blocked")
  assert_eq(trace.frame_count, 1)
  assert_eq(trace.ready_frame_count, 0)
  assert_eq(trace.blocked_frame_count, 1)
  assert_eq(trace.traction_review_count, 1)
  assert_true(trace.blocker_count >= 2)
  assert_true(
    trace.blockers.contains("frame:0:capture:capture-point-outside-support"),
  )
}

///|
test "motion hinge replay review combines motion trace and targeted replay" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let material = @moonphys.material_contact_model("test", 0.8, 0.0)
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "left-sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  let motion_0 = @moonphys.motion_frame_review(
    "motion-frame/0",
    @moonphys.lunar_environment(),
    material,
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.0, 0.0, 0.0),
    [
      @moonphys.motion_contact_evidence(
        "left-contact",
        @moonphys.support_footprint(
          "left",
          @moonphys.vec3(0.0, 0.0, 0.0),
          0.2,
          0.1,
        ),
        patch,
        @moonphys.vec3(0.0, 0.0, 80.0),
      ),
    ],
  )
  let motion_1 = @moonphys.motion_frame_review(
    "motion-frame/1",
    @moonphys.lunar_environment(),
    material,
    @moonphys.vec3(0.02, 0.0, 0.7),
    @moonphys.vec3(0.0, 0.0, 0.0),
    [
      @moonphys.motion_contact_evidence(
        "left-contact",
        @moonphys.support_footprint(
          "left",
          @moonphys.vec3(0.0, 0.0, 0.0),
          0.2,
          0.1,
        ),
        patch,
        @moonphys.vec3(4.0, 0.0, 90.0),
      ),
    ],
  )
  let motion_trace = @moonphys.motion_frame_trace_review("motion-trace", [
    motion_0, motion_1,
  ])
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
  let motor_0 = @moonphys.integrate_joint_motor_frame(
    "hinge-motor-frame/0",
    [@moonphys.joint_motor_state("shoulder", 0.0, 0.0)],
    [@moonphys.joint_motor_command("shoulder", 1.0, 0.0, 0.0)],
    [@moonphys.joint_servo_config("shoulder", 2.0, 0.0)],
    [@moonphys.joint_limit("shoulder", -2.0, 2.0, 4.0, 10.0)],
    [2.0],
    0.5,
  )
  let motor_1 = @moonphys.integrate_joint_motor_frame(
    "hinge-motor-frame/1",
    [@moonphys.joint_motor_state("shoulder", 0.25, 0.0)],
    [@moonphys.joint_motor_command("shoulder", 1.0, 0.0, 0.0)],
    [@moonphys.joint_servo_config("shoulder", 2.0, 0.0)],
    [@moonphys.joint_limit("shoulder", -2.0, 2.0, 4.0, 10.0)],
    [2.0],
    0.5,
  )
  let hinge_trace = @moonphys.rigid_body_heightfield_world_hinge_motor_trace(
    "hinge-trace",
    world,
    field,
    [motor_0, motor_1],
    note="targeted motion ingestion replay",
  )
  let footprints = [
    [
      @moonphys.support_footprint(
        "support",
        @moonphys.vec3(1.0, 0.0, 0.0),
        3.0,
        3.0,
      ),
    ],
    [
      @moonphys.support_footprint(
        "support",
        @moonphys.vec3(1.0, 0.0, 0.0),
        3.0,
        3.0,
      ),
    ],
  ]
  let targets = [
    @moonphys.rigid_body_world_center_of_mass_target(
      @moonphys.vec3(1.0, 0.0, 0.08),
      @moonphys.vec3(0.0, 0.0, 0.0),
      total_mass_kg=4.0,
      nominal_com_height_m=0.72,
    ),
    @moonphys.rigid_body_world_center_of_mass_target(
      @moonphys.vec3(1.0, 0.0, 0.08),
      @moonphys.vec3(0.0, 0.0, 0.0),
      total_mass_kg=4.0,
      nominal_com_height_m=0.72,
    ),
  ]
  let replay = @moonphys.rigid_body_heightfield_world_hinge_motor_replay_review_with_targets(
    hinge_trace, footprints, targets,
  )
  let review = @moonphys.motion_hinge_replay_review_with_replay(
    "motion-hinge-review", motion_trace, hinge_trace, replay,
  )
  assert_true(review.ready)
  assert_eq(review.status, "motion-hinge-replay-review-ready")
  assert_eq(review.motion_frame_count, 2)
  assert_eq(review.hinge_frame_count, 2)
  assert_eq(review.frame_count_delta, 0)
  assert_eq(review.driven_joint_count, 2)
  assert_true(review.resolved_hinge_constraint_count >= 2)
  assert_eq(review.motion_trace_status, "motion-frame-trace-review-ready")
  assert_eq(
    review.hinge_trace_status,
    "world-heightfield-hinge-motor-trace-driven",
  )
  assert_true(review.replay_status.contains("world-replay-review"))
  assert_true(review.max_motion_total_normal_force_n >= 90.0)
  assert_true(review.max_motion_contact_torque_nm >= 0.0)
  assert_true(review.max_motion_pressure_pa > 0.0)
  assert_true(review.max_replay_speed_mps >= 0.0)
  assert_eq(review.blocker_count, 0)
}

///|
test "motion hinge replay review blocks frame mismatches" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "left-sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  let motion_trace = @moonphys.motion_frame_trace_review(
    "single-motion-trace",
    [
      @moonphys.motion_frame_review(
        "motion-frame/0",
        @moonphys.lunar_environment(),
        @moonphys.material_contact_model("test", 0.8, 0.0),
        @moonphys.vec3(0.0, 0.0, 0.7),
        @moonphys.vec3(0.0, 0.0, 0.0),
        [
          @moonphys.motion_contact_evidence(
            "left-contact",
            @moonphys.support_footprint(
              "left",
              @moonphys.vec3(0.0, 0.0, 0.0),
              0.2,
              0.1,
            ),
            patch,
            @moonphys.vec3(0.0, 0.0, 80.0),
          ),
        ],
      ),
    ],
  )
  let body = @moonphys.rigid_body_motion_state(
    "body",
    1.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 0.2),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let world = @moonphys.rigid_body_heightfield_world(
    "idle-world",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    [@moonphys.rigid_body_world_body(body.linear)],
    motion_bodies=[
      @moonphys.rigid_body_joint_frame_body(
        body,
        @moonphys.rigid_body_inertia("body", @moonphys.vec3(1.0, 1.0, 1.0)),
      ),
    ],
  )
  let idle = @moonphys.integrate_joint_motor_frame(
    "idle",
    [],
    [],
    [],
    [],
    [],
    0.1,
  )
  let hinge_trace = @moonphys.rigid_body_heightfield_world_hinge_motor_trace(
    "idle-hinge-trace",
    world,
    field,
    [idle, idle],
  )
  let review = @moonphys.motion_hinge_replay_review(
    "blocked-review", motion_trace, hinge_trace,
  )
  assert_true(!review.ready)
  assert_eq(review.status, "motion-hinge-replay-review-blocked")
  assert_eq(review.motion_frame_count, 1)
  assert_eq(review.hinge_frame_count, 2)
  assert_true(review.blockers.contains("frame-count-mismatch"))
  assert_true(review.blockers.contains("hinge-trace:no-driven-joints"))
}
