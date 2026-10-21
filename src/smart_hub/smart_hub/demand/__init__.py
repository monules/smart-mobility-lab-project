"""S2 demand prediction: ROS-independent logic used by demand_predictor.

Everything in this subpackage is pure Python (standard library only at
runtime) so it can be unit-tested without a ROS installation and so the
offline training script and the online node share exactly the same feature
code. Only the training script needs numpy / scikit-learn.
"""

from smart_hub.demand.estimator import DemandEstimator, Prediction  # noqa: F401
from smart_hub.demand.features import (  # noqa: F401
    DemandHistory,
    extract_features,
    FEATURE_NAMES,
    FEATURE_WINDOW,
)
from smart_hub.demand.models import (  # noqa: F401
    LinearDemandModel,
    load_model,
    PersistenceModel,
)
