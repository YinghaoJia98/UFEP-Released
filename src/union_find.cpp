#include <vrmapping/union_find.h>

UF::UF(int n)
{
  count = n;
  // parent.resize(n);
  // size.resize(n);
  for (int i = 0; i < n; i++)
  {
    parent[i] = i;
    size[i] = 1;
  }
}

void UF::merge(int p, int q)
{

  int rootP = find(p);
  int rootQ = find(q);
  if (rootP == rootQ)
    return;

  // 小树接到大树下面，较平衡
  if (size[rootP] > size[rootQ])
  {
    parent[rootQ] = rootP;
    size[rootP] += size[rootQ];
  }
  else
  {
    parent[rootP] = rootQ;
    size[rootQ] += size[rootP];
  }
  count--;
}

bool UF::connected(int p, int q)
{

  int rootP = find(p);
  int rootQ = find(q);
  return rootP == rootQ;
}

int UF::getcount()
{
  return count;
}

void UF::addpoint(int newpoint_global_id)
{
  // if (newpoint_global_id == parent.size())
  // {
  //   // std::cout << "the UF addpoint might be right in logical." << std::endl;
  // }
  // else
  // {
  //   std::cout << "the UF addpoint might be wrong in logical." << std::endl;
  //   ROS_ERROR("The format of parent and size need be changed to unordered_map from vector.");
  // }
  // parent.push_back(newpoint_global_id);
  // size.push_back(newpoint_global_id);
  parent[newpoint_global_id] = newpoint_global_id;
  size[newpoint_global_id] = 1;
  count++;
}

void UF::removepoint(int newpoint_global_id)
{
  parent.erase(newpoint_global_id);
  size.erase(newpoint_global_id);
  count--;
}

int UF::find(int x)
{

  while (parent[x] != x)
  {
    // 进行路径压缩
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

bool UF::IfPointExist(int Id)
{
  if (parent.find(Id) == parent.end())
  {
    return false;
  }
  else
  {
    return true;
  }
}