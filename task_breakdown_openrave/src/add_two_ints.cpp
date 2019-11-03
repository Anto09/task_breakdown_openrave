#include <task_breakdown_openrave/add_two_ints.h>

using namespace task_breakdown_openrave;

std_msgs::Int64 AddTwoInts::add(const std_msgs::Int64& a, const std_msgs::Int64& b)
{
  std_msgs::Int64 sum;
  sum.data = a.data + b.data;
  return sum;
}