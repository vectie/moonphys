///|
pub extend PhysicsFrame with @debug.Debug::{to_repr}

///|
pub extend PhysicsFrame with Eq::{not_equal, equal}

///|
pub extend PhysicsFrame with ToJson::{to_json}

///|
pub extend PhysicsFrame with @json.FromJson::{from_json}

///|
pub extend PhysicsTrace with @debug.Debug::{to_repr}

///|
pub extend PhysicsTrace with Eq::{not_equal, equal}

///|
pub extend PhysicsTrace with ToJson::{to_json}

///|
pub extend PhysicsTrace with @json.FromJson::{from_json}
