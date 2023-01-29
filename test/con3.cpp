#include "con3/con3.h"
#include "test.h"

int main(void)
{
    con3::set<int> test;
    test.add(1);
    TEST_EQUAL(test.values().size(), 0);
    TEST_EQUAL(test.pending_changes().size(), 1);

    TEST_TRUE(test.synchronize());
    TEST_EQUAL(test.values().size(), 1);
    TEST_EQUAL(*test.values().begin(), 1);
    TEST_EQUAL(test[0], 1);

    test.add(2);
    TEST_EQUAL(test.values().size(), 1);
    TEST_EQUAL(test.pending_changes().size(), 1);
    TEST_TRUE(test.synchronize());
    TEST_EQUAL(test.values().size(), 2);
    TEST_EQUAL(*test.values().begin(), 1);
    TEST_EQUAL(test[0], 1);
    TEST_EQUAL(test[1], 2);

    test.add(3).synchronize(3);
    TEST_EQUAL(test.values().size(), 3);
    TEST_EQUAL(test.pending_changes().size(), 0);
    TEST_EQUAL(test[0], 1);
    TEST_EQUAL(test[1], 2);
    TEST_EQUAL(test[2], 3);

    test.force_add(4);
    TEST_EQUAL(test.values().size(), 4);
    TEST_EQUAL(test.pending_changes().size(), 0);
    TEST_EQUAL(test[0], 1);
    TEST_EQUAL(test[1], 2);
    TEST_EQUAL(test[2], 3);
    TEST_EQUAL(test[3], 4);

    test.add(5);
    test.remove(4);
    TEST_EQUAL(test.values().size(), 4);
    TEST_EQUAL(test.pending_changes().size(), 2);

    std::vector<int> merge = test.merge();
    TEST_EQUAL(merge.size(), 1);
    TEST_EQUAL(merge.at(0), 5);
    TEST_EQUAL(test.values().size(), 5);
    TEST_EQUAL(test[0], 1);
    TEST_EQUAL(test[1], 2);
    TEST_EQUAL(test[2], 3);
    TEST_EQUAL(test[3], 4);
    TEST_EQUAL(test[4], 5);

    std::vector<int> purge = test.purge();
    TEST_EQUAL(purge.size(), 1);
    TEST_EQUAL(purge.at(0), 4);
    TEST_EQUAL(test.values().size(), 4);
    TEST_EQUAL(test[0], 1);
    TEST_EQUAL(test[1], 2);
    TEST_EQUAL(test[2], 3);
    TEST_EQUAL(test[3], 5);

    test.force_remove(5);
    TEST_EQUAL(test.pending_changes().size(), 0);
    TEST_EQUAL(test.values().size(), 3);

    test.force_remove(5);
    TEST_EQUAL(test.pending_changes().size(), 0);
    TEST_EQUAL(test.values().size(), 3);
}