# Receding Horizon Planning

Receding horizon planning is a two-tiered approach to solving the planning problem, this approach is to be used in autonomus flying vehicles planning. First, you find a coarse global plan all the way from the start to the goal. Then, as you execute that plan, you continuously replan in a local volume around the vehicle at a higher resolution. This approach allows for fine tuning your plan on the fly, reacting to obstacles that weren't on the map or other uncertainties, like sensor errors or wind. The edge or horizon of your local planner continuously moves out in front of you and hence the name, receding horizon planning.


![Image](/images/receding_h.png)

# Features
* Breadth-First Search algorithm used to find coarse global plan.
* A* algorithm used to find a high resolution local plan in a volume around the vehicle.
