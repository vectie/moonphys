///|
pub extend CollisionContact with @debug.Debug::{to_repr}

///|
pub extend CollisionContact with Eq::{not_equal, equal}

///|
pub extend CollisionContact with ToJson::{to_json}

///|
pub extend CollisionContact with @json.FromJson::{from_json}

///|
pub extend CollisionContactManifold with @debug.Debug::{to_repr}

///|
pub extend CollisionContactManifold with Eq::{not_equal, equal}

///|
pub extend CollisionContactManifold with ToJson::{to_json}

///|
pub extend CollisionContactManifold with @json.FromJson::{from_json}

///|
pub extend CollisionManifoldResolution with @debug.Debug::{to_repr}

///|
pub extend CollisionManifoldResolution with Eq::{not_equal, equal}

///|
pub extend CollisionManifoldResolution with ToJson::{to_json}

///|
pub extend CollisionManifoldResolution with @json.FromJson::{from_json}
