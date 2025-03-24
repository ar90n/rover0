#include "utest.h"

#include "queue.hpp"
#include <memory>

// Test class for tracking object lifetime
class TestObject {
public:
    static int constructor_count;
    static int destructor_count;
    static int copy_count;
    static int move_count;
    
    int id;
    int value;
    
    TestObject(int i, int v = 0) 
        : id(i), value(v) {
        ++constructor_count;
    }
    
    TestObject(const TestObject& other)
        : id(other.id), value(other.value) {
        ++constructor_count;
        ++copy_count;
    }
    
    TestObject(TestObject&& other) noexcept
        : id(other.id), value(other.value) {
        ++constructor_count;
        ++move_count;
    }
    
    ~TestObject() {
        ++destructor_count;
    }
    
    static void reset_counters() {
        constructor_count = 0;
        destructor_count = 0;
        copy_count = 0;
        move_count = 0;
    }
    
    bool operator==(const TestObject& other) const {
        return id == other.id && value == other.value;
    }
};

int TestObject::constructor_count = 0;
int TestObject::destructor_count = 0;
int TestObject::copy_count = 0;
int TestObject::move_count = 0;

// Basic Functionality Tests

UTEST(queue, initialization) {
    FixedSizeQueue<int, 5> queue;
    ASSERT_TRUE(queue.empty());
    ASSERT_FALSE(queue.full());
    ASSERT_EQ(queue.size(), 0);
}

UTEST(queue, push_single_element) {
    FixedSizeQueue<int, 5> queue;
    bool result = queue.push(42);
    ASSERT_TRUE(result);
    ASSERT_FALSE(queue.empty());
    ASSERT_EQ(queue.size(), 1);
}

UTEST(queue, pop_single_element) {
    FixedSizeQueue<int, 5> queue;
    queue.push(42);
    auto result = queue.pop();
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(*result, 42);
    ASSERT_TRUE(queue.empty());
    ASSERT_EQ(queue.size(), 0);
}

UTEST(queue, multiple_elements) {
    FixedSizeQueue<int, 5> queue;
    queue.push(1);
    queue.push(2);
    queue.push(3);
    ASSERT_EQ(queue.size(), 3);
    
    ASSERT_EQ(*queue.pop(), 1);
    ASSERT_EQ(*queue.pop(), 2);
    ASSERT_EQ(*queue.pop(), 3);
    ASSERT_TRUE(queue.empty());
}

// Edge Cases and Boundary Conditions

UTEST(queue, full_queue) {
    FixedSizeQueue<int, 3> queue;
    ASSERT_TRUE(queue.push(1));
    ASSERT_TRUE(queue.push(2));
    ASSERT_TRUE(queue.push(3));
    ASSERT_TRUE(queue.full());
    ASSERT_FALSE(queue.push(4)); // Should fail
    ASSERT_EQ(queue.size(), 3);
    
    // Verify original elements are preserved
    ASSERT_EQ(*queue.pop(), 1);
    ASSERT_EQ(*queue.pop(), 2);
    ASSERT_EQ(*queue.pop(), 3);
}

UTEST(queue, empty_queue_pop) {
    FixedSizeQueue<int, 3> queue;
    auto result = queue.pop();
    ASSERT_FALSE(result.has_value());
    ASSERT_TRUE(queue.empty());
}

UTEST(queue, circular_buffer_wrapping) {
    FixedSizeQueue<int, 5> queue;
    
    // Fill the queue
    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(queue.push(i));
    }
    ASSERT_TRUE(queue.full());
    
    // Remove some elements
    ASSERT_EQ(*queue.pop(), 0);
    ASSERT_EQ(*queue.pop(), 1);
    ASSERT_EQ(queue.size(), 3);
    
    // Add new elements (these should wrap around in the internal buffer)
    ASSERT_TRUE(queue.push(5));
    ASSERT_TRUE(queue.push(6));
    ASSERT_TRUE(queue.full());
    
    // Verify all elements in FIFO order
    ASSERT_EQ(*queue.pop(), 2);
    ASSERT_EQ(*queue.pop(), 3);
    ASSERT_EQ(*queue.pop(), 4);
    ASSERT_EQ(*queue.pop(), 5);
    ASSERT_EQ(*queue.pop(), 6);
    ASSERT_TRUE(queue.empty());
}

UTEST(queue, capacity_limits) {
    // Test with minimum capacity
    FixedSizeQueue<int, 1> min_queue;
    ASSERT_TRUE(min_queue.push(42));
    ASSERT_TRUE(min_queue.full());
    ASSERT_FALSE(min_queue.push(43));
    ASSERT_EQ(*min_queue.pop(), 42);
    ASSERT_TRUE(min_queue.empty());
    
    // Test with larger capacity
    FixedSizeQueue<int, 100> large_queue;
    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(large_queue.push(i));
    }
    ASSERT_TRUE(large_queue.full());
    ASSERT_FALSE(large_queue.push(100));
    
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(*large_queue.pop(), i);
    }
    ASSERT_TRUE(large_queue.empty());
}

// Type Safety and Object Lifetime

UTEST(queue, primitive_types) {
    // Test with int
    {
        FixedSizeQueue<int, 3> int_queue;
        ASSERT_TRUE(int_queue.push(42));
        ASSERT_EQ(*int_queue.pop(), 42);
    }
    
    // Test with double
    {
        FixedSizeQueue<double, 3> double_queue;
        ASSERT_TRUE(double_queue.push(3.14));
        ASSERT_NEAR(*double_queue.pop(), 3.14, 0.0001);
    }
    
    // Test with another int
    {
        FixedSizeQueue<int, 3> another_int_queue;
        ASSERT_TRUE(another_int_queue.push(65)); // ASCII 'A'
        ASSERT_EQ(*another_int_queue.pop(), 65);
    }
    
    // Test with bool
    {
        FixedSizeQueue<bool, 3> bool_queue;
        ASSERT_TRUE(bool_queue.push(true));
        ASSERT_TRUE(*bool_queue.pop());
    }
}

UTEST(queue, complex_types) {
    TestObject::reset_counters();
    
    {
        FixedSizeQueue<TestObject, 3> queue;
        
        // Test push
        ASSERT_TRUE(queue.push(TestObject(1, 100)));
        ASSERT_TRUE(queue.push(TestObject(2, 200)));
        ASSERT_EQ(queue.size(), 2);
        
        // Verify construction counts (1 for original + 1 for copy in queue)
        ASSERT_EQ(TestObject::constructor_count, 4);
        
        // Test pop and verify object state
        auto obj1 = queue.pop();
        ASSERT_TRUE(obj1.has_value());
        ASSERT_EQ(obj1->id, 1);
        ASSERT_EQ(obj1->value, 100);
        
        // Destructor should be called for the popped element in the queue and the temporary objects
        ASSERT_EQ(TestObject::destructor_count, 4); // Original objects + queue copy + temporaries
        
        // Pop the second object
        auto obj2 = queue.pop();
        ASSERT_TRUE(obj2.has_value());
        ASSERT_EQ(obj2->id, 2);
        ASSERT_EQ(obj2->value, 200);
        
        // Queue should be empty now
        ASSERT_TRUE(queue.empty());
    }
    
    // After queue goes out of scope, all remaining objects should be destroyed
    ASSERT_EQ(TestObject::constructor_count, TestObject::destructor_count);
}

UTEST(queue, move_semantics) {
    TestObject::reset_counters();
    
    {
        // Test with a class that tracks moves
        FixedSizeQueue<TestObject, 3> queue;
        
        // Create objects outside the queue
        TestObject obj1(1, 100);
        TestObject obj2(2, 200);
        
        // Push objects into the queue (will be copied due to const& parameter)
        ASSERT_TRUE(queue.push(obj1));
        ASSERT_TRUE(queue.push(obj2));
        
        // Verify original objects still exist
        ASSERT_EQ(obj1.id, 1);
        ASSERT_EQ(obj1.value, 100);
        
        // Pop and verify
        auto popped1 = queue.pop();
        ASSERT_TRUE(popped1.has_value());
        ASSERT_EQ(popped1->id, 1);
        ASSERT_EQ(popped1->value, 100);
        
        auto popped2 = queue.pop();
        ASSERT_TRUE(popped2.has_value());
        ASSERT_EQ(popped2->id, 2);
        ASSERT_EQ(popped2->value, 200);
        
        // Check copy counts
        ASSERT_TRUE(TestObject::copy_count > 0);
    }
    
    // Verify all objects were properly destroyed
    ASSERT_EQ(TestObject::constructor_count, TestObject::destructor_count);
}

// Stress and Performance Tests

UTEST(queue, queue_cycling) {
    FixedSizeQueue<int, 10> queue;
    
    // Repeatedly fill and empty the queue
    for (int cycle = 0; cycle < 5; ++cycle) {
        // Fill
        for (int i = 0; i < 10; ++i) {
            ASSERT_TRUE(queue.push(i + cycle * 100));
        }
        ASSERT_TRUE(queue.full());
        
        // Empty
        for (int i = 0; i < 10; ++i) {
            auto val = queue.pop();
            ASSERT_TRUE(val.has_value());
            ASSERT_EQ(*val, i + cycle * 100);
        }
        ASSERT_TRUE(queue.empty());
    }
}

UTEST(queue, mixed_operations) {
    FixedSizeQueue<int, 5> queue;
    
    // Interleave push and pop operations
    ASSERT_TRUE(queue.push(1));
    ASSERT_TRUE(queue.push(2));
    ASSERT_EQ(*queue.pop(), 1);
    ASSERT_TRUE(queue.push(3));
    ASSERT_EQ(*queue.pop(), 2);
    ASSERT_TRUE(queue.push(4));
    ASSERT_TRUE(queue.push(5));
    ASSERT_TRUE(queue.push(6));
    ASSERT_EQ(queue.size(), 4);
    
    // Verify remaining elements
    ASSERT_EQ(*queue.pop(), 3);
    ASSERT_EQ(*queue.pop(), 4);
    ASSERT_EQ(*queue.pop(), 5);
    ASSERT_EQ(*queue.pop(), 6);
    ASSERT_TRUE(queue.empty());
}

// Test destructor behavior
UTEST(queue, destructor_cleanup) {
    TestObject::reset_counters();
    
    {
        FixedSizeQueue<TestObject, 3> queue;
        queue.push(TestObject(1, 100));
        queue.push(TestObject(2, 200));
        queue.push(TestObject(3, 300));
        
        // Queue goes out of scope here, destructor should clean up all elements
    }
    
    // Verify all objects were properly destroyed
    ASSERT_EQ(TestObject::constructor_count, TestObject::destructor_count);
}
