///|
pub extend CollisionShapeKind with @debug.Debug::{to_repr}

///|
pub extend CollisionShapeKind with Eq::{not_equal, equal}

///|
pub extend CollisionShapeKind with ToJson::{to_json}

///|
pub extend CollisionShapeKind with @json.FromJson::{from_json}

///|
pub extend CollisionShape with @debug.Debug::{to_repr}

///|
pub extend CollisionShape with Eq::{not_equal, equal}

///|
pub extend CollisionShape with ToJson::{to_json}

///|
pub extend CollisionShape with @json.FromJson::{from_json}

///|
pub extend CollisionBounds with @debug.Debug::{to_repr}

///|
pub extend CollisionBounds with Eq::{not_equal, equal}

///|
pub extend CollisionBounds with ToJson::{to_json}

///|
pub extend CollisionBounds with @json.FromJson::{from_json}

///|
pub extend CollisionPairProbe with @debug.Debug::{to_repr}

///|
pub extend CollisionPairProbe with Eq::{not_equal, equal}

///|
pub extend CollisionPairProbe with ToJson::{to_json}

///|
pub extend CollisionPairProbe with @json.FromJson::{from_json}
