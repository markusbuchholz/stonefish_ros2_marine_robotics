import numpy as np

def map_coordinates(x, y):
    # Map y from [-5, 5] to [0, 10]
    new_x = (y + 5) * (10 / 10)

    # Map x from [-5, 5] to [5, -5]
    new_y = (x + 5) * (-10 / 10) + 5

    return new_x *10 , new_y *10

def reduce_waypoints(waypoints, num_points=5):
    if len(waypoints) <= num_points:
        return waypoints

    reduced_waypoints = [waypoints[0]]  # Start with the first point

    interval = len(waypoints) // (num_points - 1)  # Calculate interval
    for i in range(1, num_points - 1):
        reduced_waypoints.append(waypoints[i * interval])

    reduced_waypoints.append(waypoints[-1])  # End with the last point

    return reduced_waypoints

# Example set of waypoints
waypoints = [
(-0.42, -4.08),
(-0.36512379754858487, -3.8876757883039392),
(-0.2173530089631537, -3.7529026676869566),
(-0.08416919900594072, -3.6036978662719856),
(0.021739907950316384, -3.4340415635192496),
(0.15367999766005244, -3.2837358326658093),
(0.11506261187146269, -3.0874995018938156),
(-0.31027653013664425, 0.3271899119782511),
(-0.34713055356022937, 0.5237650400018489),
(-0.45639801850690015, 0.6912780875656971),
(-1.11149811636374, 1.140192625097207),
(-1.2971206811936886, 1.214652435907815),
(-1.378418239778065, 1.3973836783602755),
(-2.626566284202259, 1.8593481895602668),
(-2.695833736656197, 1.6717261758450808),
(-2.8108379680311657, 1.5080984680268368),
(-2.7906134638500664, 1.3091236720585315),
(-2.7970912283914444, 1.10922860316851),
(-2.99656022051965, 1.1237929836688172),
(-3.116706809397812, 0.9639030303229551),
(-3.2425662781744036, 0.8084701054991772),
(-3.3003336813923756, 0.6169944487554224),
(-3.34, 0.46)
]

# Reduce waypoints
reduced_waypoints = reduce_waypoints(waypoints)

# Map the reduced waypoints
mapped_waypoints = [map_coordinates(x, y) for x, y in reduced_waypoints]

print("Reduced and Mapped Waypoints:")
for wp in mapped_waypoints:
    print(wp)
