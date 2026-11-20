///|
pub extend RigidBodyWorldDynamicSupportFrame with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldDynamicSupportFrame with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldDynamicSupportFrame with ToJson::{to_json}

///|
pub extend RigidBodyWorldDynamicSupportFrame with @json.FromJson::{from_json}

///|
pub extend RigidBodyWorldDynamicSupportTrace with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldDynamicSupportTrace with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldDynamicSupportTrace with ToJson::{to_json}

///|
pub extend RigidBodyWorldDynamicSupportTrace with @json.FromJson::{from_json}
