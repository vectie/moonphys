///|
pub extend PhysicsEnvironment with @debug.Debug::{to_repr}

///|
pub extend PhysicsEnvironment with Eq::{not_equal, equal}

///|
pub extend PhysicsEnvironment with ToJson::{to_json}

///|
pub extend PhysicsEnvironment with @json.FromJson::{from_json}
