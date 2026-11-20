///|
pub extend ModelMetadataInventory with @debug.Debug::{to_repr}

///|
pub extend ModelMetadataInventory with Eq::{not_equal, equal}

///|
pub extend ModelMetadataInventory with ToJson::{to_json}

///|
pub extend ModelMetadataInventory with @json.FromJson::{from_json}

///|
pub extend PhysicalModelReadiness with @debug.Debug::{to_repr}

///|
pub extend PhysicalModelReadiness with Eq::{not_equal, equal}

///|
pub extend PhysicalModelReadiness with ToJson::{to_json}

///|
pub extend PhysicalModelReadiness with @json.FromJson::{from_json}
