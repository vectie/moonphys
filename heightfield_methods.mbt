///|
pub extend Heightfield with @debug.Debug::{to_repr}

///|
pub extend Heightfield with Eq::{not_equal, equal}

///|
pub extend Heightfield with ToJson::{to_json}

///|
pub extend Heightfield with @json.FromJson::{from_json}

///|
pub extend HeightfieldSample with @debug.Debug::{to_repr}

///|
pub extend HeightfieldSample with Eq::{not_equal, equal}

///|
pub extend HeightfieldSample with ToJson::{to_json}

///|
pub extend HeightfieldSample with @json.FromJson::{from_json}
