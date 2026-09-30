# What one drone tells the rest of the swarm, ten times a second.
#
# This is the entire interface between drones. Everything swarm_comms does that
# swarm_naive cannot do is built out of these seven fields, so it is worth
# understanding why each one is here and why nothing else is.
#
# It is published on /swarm/state, ONE topic shared by every drone, deliberately
# outside any px4_ namespace. On real hardware every aircraft publishes its
# autopilot on bare /fmu/..., so there is no namespace left to tell drones apart
# and drone-to-drone traffic cannot live under those names. The `slot` field is
# what identifies the sender instead, exactly as it would in the field.
#
# DESIGNED SO THAT LOSING A MESSAGE DOES NOT MATTER
# -------------------------------------------------
# Every field is a LEVEL, never an EDGE. Nothing here says "I have just become
# ready"; it says "I am ready", over and over, ten times a second, for as long
# as it stays true. A receiver that misses a message learns the same thing from
# the next one 100 ms later.
#
# That is what lets the topic run BEST_EFFORT, which is what you want over a
# real radio link: a retransmitted position that is half a second old is worse
# than the one that arrived on time. Had `ready` been an announcement rather
# than a state, a single dropped packet would have deadlocked the whole swarm.

uint64 stamp_us     # sender's clock, microseconds, to match PX4's convention.
                    # The receiver compares it with nothing: it measures age by
                    # its OWN arrival time, because two drones do not share a
                    # clock. This field is here for logs and for the day they do.

uint8 slot          # which drone this is, 0 to count-1. The only identity that
                    # survives the trip to real hardware.

bool ready          # "I am at my station and willing to start".

bool done           # "I have finished the pattern and am going home". Each
                    # drone watches the one ahead of it for this, so the swarm
                    # peels off in order without anyone coordinating it.

float32 north_m     # Position in the SHARED SWARM FRAME, not in the sender's
float32 east_m      # own PX4 frame. The sender converts before publishing,
float32 down_m      # because a position measured from the sender's own pad is
                    # meaningless to anybody else. NED, so down is positive
                    # downward. This conversion is the shared-frame problem, and
                    # it is the reason this message exists at all.
