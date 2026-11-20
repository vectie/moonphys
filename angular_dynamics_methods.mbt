///|
pub extend RigidBodyInertia with @debug.Debug::{to_repr}

///|
pub extend RigidBodyInertia with Eq::{not_equal, equal}

///|
pub extend RigidBodyInertia with ToJson::{to_json}

///|
pub extend RigidBodyInertia with @json.FromJson::{from_json}

///|
pub extend MassPropertyElement with @debug.Debug::{to_repr}

///|
pub extend MassPropertyElement with Eq::{not_equal, equal}

///|
pub extend MassPropertyElement with ToJson::{to_json}

///|
pub extend MassPropertyElement with @json.FromJson::{from_json}

///|
pub extend CompositeMassProperties with @debug.Debug::{to_repr}

///|
pub extend CompositeMassProperties with Eq::{not_equal, equal}

///|
pub extend CompositeMassProperties with ToJson::{to_json}

///|
pub extend CompositeMassProperties with @json.FromJson::{from_json}

///|
pub extend RigidBodyAngularState with @debug.Debug::{to_repr}

///|
pub extend RigidBodyAngularState with Eq::{not_equal, equal}

///|
pub extend RigidBodyAngularState with ToJson::{to_json}

///|
pub extend RigidBodyAngularState with @json.FromJson::{from_json}

///|
pub extend RigidBodyAngularStep with @debug.Debug::{to_repr}

///|
pub extend RigidBodyAngularStep with Eq::{not_equal, equal}

///|
pub extend RigidBodyAngularStep with ToJson::{to_json}

///|
pub extend RigidBodyAngularStep with @json.FromJson::{from_json}
