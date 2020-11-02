# Message representing an LGMath Transformation or TransformationWithCovariance

# Lie group vector
float64[] xi

# Flattened covariance matrix
float64[] cov

# Flag indicating if the covariance of this transform has been explicitly set
bool cov_set