///|
pub extend RigidBodyState with @debug.Debug::{to_repr}

///|
pub extend RigidBodyState with Eq::{not_equal, equal}

///|
pub extend RigidBodyState with ToJson::{to_json}

///|
pub extend RigidBodyState with @json.FromJson::{from_json}

///|
pub extend MaterialContactModel with @debug.Debug::{to_repr}

///|
pub extend MaterialContactModel with Eq::{not_equal, equal}

///|
pub extend MaterialContactModel with ToJson::{to_json}

///|
pub extend MaterialContactModel with @json.FromJson::{from_json}

///|
pub extend HeightfieldCollision with @debug.Debug::{to_repr}

///|
pub extend HeightfieldCollision with Eq::{not_equal, equal}

///|
pub extend HeightfieldCollision with ToJson::{to_json}

///|
pub extend HeightfieldCollision with @json.FromJson::{from_json}

///|
pub extend ContactResolution with @debug.Debug::{to_repr}

///|
pub extend ContactResolution with Eq::{not_equal, equal}

///|
pub extend ContactResolution with ToJson::{to_json}

///|
pub extend ContactResolution with @json.FromJson::{from_json}
