## \brief Stores a pair of indices defining a range
## \details This can be either a stream index or a time index, as we store
##          times as unint64 nanoseconds since epoch.  A NULL index may be
##          represented by not setting either value.

#
uint64 idx1 0
#
uint64 idx2 0