///|
pub extend RigidBodyWorldBodyTarget with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldBodyTarget with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldBodyTarget with ToJson::{to_json}

///|
pub extend RigidBodyWorldBodyTarget with @json.FromJson::{from_json}

///|
pub extend RigidBodyWorldBodyTargetStep with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldBodyTargetStep with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldBodyTargetStep with ToJson::{to_json}

///|
pub extend RigidBodyWorldBodyTargetStep with @json.FromJson::{from_json}

///|
pub extend RigidBodyWorldBodyTargetFrame with @debug.Debug::{to_repr}

///|
pub extend RigidBodyWorldBodyTargetFrame with Eq::{not_equal, equal}

///|
pub extend RigidBodyWorldBodyTargetFrame with ToJson::{to_json}

///|
pub extend RigidBodyWorldBodyTargetFrame with @json.FromJson::{from_json}
