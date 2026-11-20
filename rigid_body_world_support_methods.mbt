///|
pub extend RigidBodyWorldSupportFrame with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldSupportFrame with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldSupportFrame with ToJson::{to_json}

///|
pub extend RigidBodyWorldSupportFrame with @json.FromJson::{from_json}

///|
pub extend RigidBodyWorldSupportTrace with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldSupportTrace with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldSupportTrace with ToJson::{to_json}

///|
pub extend RigidBodyWorldSupportTrace with @json.FromJson::{from_json}
