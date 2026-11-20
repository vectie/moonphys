///|
test "vec3 math is deterministic" {
  let a = @moonphys.vec3(1.0, 2.0, 3.0)
  let b = @moonphys.vec3(4.0, 5.0, 6.0)
  assert_true(a.add(b) == @moonphys.vec3(5.0, 7.0, 9.0))
  assert_true(b.sub(a) == @moonphys.vec3(3.0, 3.0, 3.0))
  assert_true(a.scale(2.0) == @moonphys.vec3(2.0, 4.0, 6.0))
  assert_true(
    @moonphys.vec3(1.0, 0.0, 0.0).cross(@moonphys.vec3(0.0, 1.0, 0.0)) ==
    @moonphys.vec3(0.0, 0.0, 1.0),
  )
  inspect(a.dot(b), content="32")
}

///|
fn nearly_equal(a : Double, b : Double) -> Bool {
  let delta = a - b
  let absolute = if delta < 0.0 { 0.0 - delta } else { delta }
  absolute < 0.0000001
}

///|
test "heightfield sample wraps along x" {
  let field = @moonphys.heightfield("test-field", 1, 2, 10.0, [1.0, 4.0])
  let sample = @moonphys.sample_heightfield(field, 21.0, 0.0)
  assert_eq(sample.col, 0)
  assert_true(nearly_equal(sample.elevation_m, 1.3))
  inspect(sample.local_grade_x, content="0.3")
}

///|
test "heightfield sample bilinearly interpolates elevation and normal" {
  let field = @moonphys.heightfield("slope-field", 2, 2, 1.0, [
    0.0, 1.0, 2.0, 3.0,
  ])
  let sample = @moonphys.sample_heightfield(field, 0.5, -0.5)
  assert_eq(sample.row, 0)
  assert_eq(sample.col, 0)
  assert_true(nearly_equal(sample.elevation_m, 1.5))
  assert_true(nearly_equal(sample.local_grade_x, 1.0))
  assert_true(nearly_equal(sample.local_grade_y, 2.0))
  assert_true(sample.surface_normal.x < 0.0)
  assert_true(sample.surface_normal.y < 0.0)
  assert_true(sample.surface_normal.z > 0.0)
  assert_true(nearly_equal(sample.surface_normal.length(), 1.0))
}

///|
test "contact probe detects clearance" {
  let field = @moonphys.heightfield("test-field", 1, 1, 10.0, [2.0])
  let probe = @moonphys.heightfield_contact_probe(
    field,
    "foot",
    @moonphys.vec3(0.0, 0.0, 2.5),
  )
  assert_true(!probe.in_contact)
  inspect(probe.clearance_m, content="0.5")
  assert_true(probe.surface_normal == @moonphys.vec3(0.0, 0.0, 1.0))
}

///|
test "heightfield contact patch samples rectangular sole" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  assert_eq(patch.sample_count, 5)
  assert_eq(patch.contact_count, 5)
  assert_eq(patch.status, "patch-contact")
  assert_true(nearly_equal(patch.min_clearance_m, 0.0))
  assert_true(nearly_equal(patch.max_clearance_m, 0.0))
  assert_true(patch.average_surface_normal == @moonphys.vec3(0.0, 0.0, 1.0))
}

///|
test "heightfield oriented contact patch follows sloped terrain" {
  let field = @moonphys.heightfield("x-slope-field", 2, 2, 1.0, [
    0.0, 1.0, 0.0, 1.0,
  ])
  let patch = @moonphys.heightfield_oriented_contact_patch(
    field,
    "sole",
    @moonphys.vec3(0.5, -0.5, 0.5),
    @moonphys.vec3(1.0, 0.0, 1.0),
    @moonphys.vec3(0.0, 1.0, 0.0),
    0.25,
    0.1,
  )
  assert_eq(patch.sample_count, 5)
  assert_eq(patch.contact_count, 5)
  assert_eq(patch.status, "patch-contact")
  assert_true(nearly_equal(patch.min_clearance_m, 0.0))
  assert_true(nearly_equal(patch.max_clearance_m, 0.0))
}

///|
test "contact patch load distributes normal force over samples" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  let load = @moonphys.contact_patch_load_assessment(patch, 100.0)
  assert_eq(load.sample_count, 5)
  assert_eq(load.contact_count, 5)
  assert_eq(load.loaded_sample_count, 5)
  assert_true(nearly_equal(load.center_of_pressure.x, 0.0))
  assert_true(nearly_equal(load.center_of_pressure.y, 0.0))
  assert_true(nearly_equal(load.samples[0].normal_force_n, 20.0))
  assert_true(nearly_equal(load.contact_area_m2, 0.08))
  assert_true(nearly_equal(load.max_pressure_pa, 1250.0))
  assert_eq(load.status, "patch-load-distributed")
}

///|
test "contact patch wrench integrates sample loads" {
  let field = @moonphys.heightfield("flat-field", 1, 1, 1.0, [0.0])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "sole",
    @moonphys.vec3(0.0, 0.0, 0.0),
    0.2,
    0.1,
  )
  let load = @moonphys.contact_patch_load_assessment(patch, 100.0)
  let wrench = @moonphys.contact_patch_wrench_assessment(
    load,
    @moonphys.vec3(0.1, 0.0, 0.0),
    @moonphys.vec3(0.0, 0.0, 2.0),
  )
  assert_true(nearly_equal(wrench.total_force_n.x, 0.0))
  assert_true(nearly_equal(wrench.total_force_n.y, 0.0))
  assert_true(nearly_equal(wrench.total_force_n.z, 100.0))
  assert_true(nearly_equal(wrench.total_torque_nm.x, 0.0))
  assert_true(nearly_equal(wrench.total_torque_nm.y, 10.0))
  assert_true(nearly_equal(wrench.total_torque_nm.z, 0.0))
  assert_eq(wrench.status, "patch-wrench-resolved")
}

///|
test "contact patch load flags partial contact pressure" {
  let field = @moonphys.heightfield("slope-field", 2, 2, 1.0, [
    0.0, 1.0, 0.0, 1.0,
  ])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "sole",
    @moonphys.vec3(0.5, -0.5, 0.5),
    0.25,
    0.1,
  )
  let load = @moonphys.contact_patch_load_assessment(patch, 100.0)
  assert_true(load.contact_count > 0)
  assert_true(load.loaded_sample_count < load.sample_count)
  assert_true(load.max_pressure_pa > load.average_pressure_pa)
  assert_eq(load.status, "patch-load-partial-contact-review")
}

///|
test "heightfield contact patch exposes partial sole penetration" {
  let field = @moonphys.heightfield("slope-field", 2, 2, 1.0, [
    0.0, 1.0, 0.0, 1.0,
  ])
  let patch = @moonphys.heightfield_contact_patch(
    field,
    "sole",
    @moonphys.vec3(0.5, -0.5, 0.5),
    0.25,
    0.1,
  )
  assert_eq(patch.sample_count, 5)
  assert_true(patch.contact_count > 0)
  assert_true(patch.contact_count < patch.sample_count)
  assert_true(patch.min_clearance_m < 0.0)
  assert_true(patch.max_clearance_m > 0.0)
  assert_eq(patch.status, "patch-penetration-review")
  assert_true(patch.average_surface_normal.x < 0.0)
}

///|
test "kinematic integration applies acceleration" {
  let state = @moonphys.kinematic_body_state(
    "body",
    @moonphys.vec3(0.0, 0.0, 1.0),
    @moonphys.vec3(1.0, 0.0, 0.0),
  )
  let stepped = @moonphys.integrate_kinematic(state, {
    dt_s: 0.5,
    acceleration: @moonphys.vec3(0.0, 0.0, -2.0),
  })
  assert_true(stepped.position == @moonphys.vec3(0.5, 0.0, 0.5))
  assert_true(stepped.velocity == @moonphys.vec3(1.0, 0.0, -1.0))
}

///|
test "rigid body gravity integration uses semi implicit step" {
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.25,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(1.0, 0.0, 0.0),
  )
  let stepped = @moonphys.integrate_rigid_body_gravity(
    state,
    @moonphys.lunar_environment(),
    0.5,
  )
  assert_true(stepped.position == @moonphys.vec3(0.5, 0.0, 1.59375))
  assert_true(stepped.velocity == @moonphys.vec3(1.0, 0.0, -0.8125))
}

///|
test "heightfield collision reports penetration and tangent speed" {
  let field = @moonphys.heightfield("test-field", 1, 1, 10.0, [1.0])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.2,
    @moonphys.vec3(0.0, 0.0, 1.1),
    @moonphys.vec3(3.0, 4.0, -2.0),
  )
  let collision = @moonphys.heightfield_collision(field, state)
  assert_true(collision.in_contact)
  assert_true(nearly_equal(collision.clearance_m, -0.1))
  assert_true(nearly_equal(collision.penetration_depth_m, 0.1))
  assert_true(nearly_equal(collision.tangent_speed_mps, 5.0))
  assert_eq(collision.status, "heightfield-contact")
}

///|
test "heightfield contact response clamps and damps velocity" {
  let field = @moonphys.heightfield("test-field", 1, 1, 10.0, [1.0])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.2,
    @moonphys.vec3(0.0, 0.0, 1.1),
    @moonphys.vec3(2.0, 0.0, -3.0),
  )
  let material = @moonphys.material_contact_model("test-regolith", 0.25, 0.1)
  let resolved = @moonphys.resolve_heightfield_contact(field, state, material)
  assert_eq(resolved.status, "contact-resolved")
  assert_true(resolved.after.position == @moonphys.vec3(0.0, 0.0, 1.2))
  assert_true(nearly_equal(resolved.after.velocity.x, 1.5))
  assert_true(nearly_equal(resolved.after.velocity.y, 0.0))
  assert_true(nearly_equal(resolved.after.velocity.z, 0.3))
}

///|
test "heightfield contact response follows slope normal" {
  let field = @moonphys.heightfield("slope-field", 2, 2, 1.0, [
    0.0, 1.0, 0.0, 1.0,
  ])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.1,
    @moonphys.vec3(0.5, -0.5, 0.55),
    @moonphys.vec3(0.0, 0.0, -1.0),
  )
  let resolved = @moonphys.resolve_heightfield_contact(
    field,
    state,
    @moonphys.material_contact_model("test-regolith", 1.0, 0.0),
  )
  assert_eq(resolved.status, "contact-resolved")
  assert_true(resolved.collision.surface_normal.x < 0.0)
  assert_true(resolved.collision.surface_normal.z > 0.0)
  assert_true(resolved.collision.normal_velocity_mps > -1.0)
  assert_true(resolved.after.position.x < state.position.x)
  assert_true(resolved.after.position.z > state.position.z)
  assert_true(nearly_equal(resolved.after.velocity.length(), 0.0))
}

///|
test "heightfield contact response leaves clear body unchanged" {
  let field = @moonphys.heightfield("test-field", 1, 1, 10.0, [1.0])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.2,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(2.0, 0.0, -3.0),
  )
  let resolved = @moonphys.resolve_heightfield_contact(
    field,
    state,
    @moonphys.lunar_regolith_contact_model(),
  )
  assert_eq(resolved.status, "clear")
  assert_true(resolved.after == state)
  assert_true(!resolved.collision.in_contact)
}

///|
test "rigid body heightfield trace is deterministic" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.2,
    @moonphys.vec3(0.0, 0.0, 0.25),
    @moonphys.vec3(0.0, 0.0, -1.0),
  )
  let trace = @moonphys.rigid_body_heightfield_trace(
    "trace",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    state,
    0.2,
    4,
    @moonphys.vec3(0.0, 0.0, 0.0),
    "test trace",
  )
  let replay = @moonphys.rigid_body_heightfield_trace(
    "trace",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    state,
    0.2,
    4,
    @moonphys.vec3(0.0, 0.0, 0.0),
    "test trace",
  )
  assert_true(trace == replay)
  assert_eq(trace.frame_count, 4)
  assert_eq(trace.status, "deterministic-replay")
  assert_eq(trace.frames[0].status, "contact-resolved")
  assert_true(nearly_equal(trace.final_state.position.z, 0.2))
}

///|
test "rigid body heightfield trace keeps stable prefixes" {
  let field = @moonphys.heightfield("flat", 1, 1, 1.0, [0.0])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.2,
    @moonphys.vec3(0.0, 0.0, 0.25),
    @moonphys.vec3(0.0, 0.0, -1.0),
  )
  let prefix = @moonphys.rigid_body_heightfield_trace(
    "trace",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    state,
    0.2,
    2,
    @moonphys.vec3(0.0, 0.0, 0.0),
    "test trace",
  )
  let longer = @moonphys.rigid_body_heightfield_trace(
    "trace",
    field,
    @moonphys.lunar_regolith_contact_model(),
    @moonphys.lunar_environment(),
    state,
    0.2,
    4,
    @moonphys.vec3(0.0, 0.0, 0.0),
    "test trace",
  )
  assert_true(prefix.frames[0] == longer.frames[0])
  assert_true(prefix.frames[1] == longer.frames[1])
  assert_true(prefix.final_state == longer.frames[1].after)
}
