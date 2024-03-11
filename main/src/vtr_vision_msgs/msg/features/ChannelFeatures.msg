# the features seen by a rig for a specific channel
string name    # the channel name (e.g., grey, cc-dessert, cc-forest)
Features[] cameras  # the features from each camera for this channel
Match[] rig_matches     # matches across cameras (e.g., stereo)
bool fully_matched
