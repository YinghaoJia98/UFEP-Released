#ifndef IDPOOL_H_
#define IDPOOL_H_
#include <iostream>
#include <set>

class IdPool
{

public:
    // constructor to initialize id_pool with integers from 0 to 100000
    IdPool();
    IdPool(int IdMax);

    // 1. Function to extract the smallest id from the pool
    int extractId();

    // 2. Function to add an id to the pool
    void addId(int id);
    void removeId(int id);

private:
    std::set<int> id_pool;
};

#endif /* IDPOOL_H_ */
