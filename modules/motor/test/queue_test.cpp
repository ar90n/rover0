#include "utest.h"

#include "queue.hpp"

UTEST(queue, test_initial_state)
{
  FixedSizeQueue<uint8_t, 3> myQueue;

  ASSERT_TRUE(myQueue.empty());
  ASSERT_TRUE(!myQueue.full());
  ASSERT_EQ(myQueue.size(), 0);
}

UTEST(queue, test_pushing_elements)
{
  FixedSizeQueue<uint8_t, 2> myQueue;

  ASSERT_TRUE(myQueue.push(10));
  ASSERT_TRUE(!myQueue.empty());
  ASSERT_TRUE(!myQueue.full());
  ASSERT_EQ(myQueue.size(), 1);

  ASSERT_TRUE(myQueue.push(30));
  ASSERT_TRUE(myQueue.full());
  ASSERT_EQ(myQueue.size(), 2);

  ASSERT_FALSE(myQueue.push(40));
}

UTEST(queue, test_popping_elements)
{
  FixedSizeQueue<uint8_t, 2> myQueue;
  ASSERT_TRUE(myQueue.push(10));
  ASSERT_TRUE(myQueue.push(30));

  ASSERT_EQ(myQueue.pop().value(), 10);
  ASSERT_EQ(myQueue.size(), 1);

  ASSERT_EQ(myQueue.pop().value(), 30);
  ASSERT_TRUE(myQueue.empty());
  ASSERT_EQ(myQueue.size(), 0);

  ASSERT_FALSE(myQueue.pop().has_value());
}

UTEST(queue, test_refilling_queue)
{
  FixedSizeQueue<uint16_t, 2> myQueue;

  myQueue.push(100);
  myQueue.push(200);
  myQueue.push(300);
  ASSERT_TRUE(myQueue.full());
  ASSERT_EQ(myQueue.size(), 2);

  ASSERT_EQ(myQueue.pop().value(), 100);
  ASSERT_EQ(myQueue.size(), 1);
}

UTEST(queue, test_queue_destructor)
{
  FixedSizeQueue<uint8_t, 2> myQueue;

  myQueue.push(10);
  myQueue.push(20);
  ASSERT_EQ(myQueue.size(), 2);

  myQueue.~FixedSizeQueue();
  ASSERT_TRUE(myQueue.empty());
  ASSERT_EQ(myQueue.size(), 0);
}

UTEST(queue, test_complex_type)
{
  struct ComplexType
  {
    int const a;
    int b;
  };

  FixedSizeQueue<ComplexType, 2> myQueue;

  myQueue.push(ComplexType{10, 20});
  myQueue.push(ComplexType{30, 40});
  ASSERT_TRUE(myQueue.full());
  ASSERT_EQ(myQueue.size(), 2);

  ASSERT_EQ(myQueue.pop().value().a, 10);
  ASSERT_EQ(myQueue.size(), 1);
}