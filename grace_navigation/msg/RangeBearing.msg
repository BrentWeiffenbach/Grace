float32 range           # range data [m]
                        # (Note: values < range_min or > range_max
                        # should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
float32 elevation       # elevation data [m]
                        # The exact same as range but for the y axis (of a 2d plane)
float32 bearing
int16 id
string obj_class
