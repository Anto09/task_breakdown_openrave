#ifndef TASK_BREAKDOWN_OPENRAVE_ADD_TWO_INTS_H
#define TASK_BREAKDOWN_OPENRAVE_ADD_TWO_INTS_H

#include <std_msgs/Int64.h>
#include <sdf_tools/SDF.h>

namespace task_breakdown_openrave {

class AddTwoInts
{
  public:
    std_msgs::Int64 add(const std_msgs::Int64& a, const std_msgs::Int64& b);
};

} // namespace python_bindings_tutorial

#endif // TASK_BREAKDOWN_OPENRAVE_ADD_TWO_INTS_H
