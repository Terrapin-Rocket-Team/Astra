#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "Utils/CircBuffer.h"

namespace test_circular_buffer {

CircBuffer<int> *intBuffer = nullptr;
CircBuffer<double> *doubleBuffer = nullptr;
CircBuffer<float> *floatBuffer = nullptr;

void suite_setUp(void)
{
    intBuffer = new CircBuffer<int>(5);
    doubleBuffer = new CircBuffer<double>(5);
    floatBuffer = new CircBuffer<float>(5);
}

void suite_tearDown(void)
{
    delete intBuffer;
    delete doubleBuffer;
    delete floatBuffer;
    intBuffer = nullptr;
    doubleBuffer = nullptr;
    floatBuffer = nullptr;
}

void local_setUp(void)
{
    if (intBuffer) intBuffer->clear();
    if (doubleBuffer) doubleBuffer->clear();
    if (floatBuffer) floatBuffer->clear();
}

void local_tearDown(void)
{
}

void test_constructor_creates_empty_buffer()
{
    local_setUp();
    CircBuffer<int> buf(10);
    TEST_ASSERT_TRUE(buf.isEmpty());
    TEST_ASSERT_EQUAL(0, buf.getCount());
    TEST_ASSERT_EQUAL(10, buf.getSize());
    local_tearDown();
}

void test_constructor_different_sizes()
{
    local_setUp();
    CircBuffer<int> buf1(1);
    CircBuffer<int> buf5(5);
    CircBuffer<int> buf100(100);

    TEST_ASSERT_EQUAL(1, buf1.getSize());
    TEST_ASSERT_EQUAL(5, buf5.getSize());
    TEST_ASSERT_EQUAL(100, buf100.getSize());
    local_tearDown();
}

void test_copy_constructor_empty_buffer()
{
    local_setUp();
    CircBuffer<int> original(5);
    CircBuffer<int> copy(original);

    TEST_ASSERT_EQUAL(original.getSize(), copy.getSize());
    TEST_ASSERT_EQUAL(original.getCount(), copy.getCount());
    TEST_ASSERT_TRUE(copy.isEmpty());
    local_tearDown();
}

void test_copy_constructor_populated_buffer()
{
    local_setUp();
    CircBuffer<int> original(5);
    original.push(1);
    original.push(2);
    original.push(3);

    CircBuffer<int> copy(original);

    TEST_ASSERT_EQUAL(original.getSize(), copy.getSize());
    TEST_ASSERT_EQUAL(original.getCount(), copy.getCount());
    TEST_ASSERT_EQUAL(1, copy[0]);
    TEST_ASSERT_EQUAL(2, copy[1]);
    TEST_ASSERT_EQUAL(3, copy[2]);
    local_tearDown();
}

void test_copy_constructor_full_buffer()
{
    local_setUp();
    CircBuffer<int> original(3);
    original.push(10);
    original.push(20);
    original.push(30);

    CircBuffer<int> copy(original);

    TEST_ASSERT_TRUE(copy.isFull());
    TEST_ASSERT_EQUAL(10, copy.pop());
    TEST_ASSERT_EQUAL(20, copy.pop());
    TEST_ASSERT_EQUAL(30, copy.pop());
    local_tearDown();
}

void test_copy_constructor_deep_copy()
{
    local_setUp();
    CircBuffer<int> original(3);
    original.push(1);
    original.push(2);

    CircBuffer<int> copy(original);

    // Modify original
    original.push(3);
    original.push(4);

    // Copy should be unchanged
    TEST_ASSERT_EQUAL(2, copy.getCount());
    TEST_ASSERT_EQUAL(1, copy[0]);
    TEST_ASSERT_EQUAL(2, copy[1]);
    local_tearDown();
}

void test_copy_assignment_operator()
{
    local_setUp();
    CircBuffer<int> original(5);
    original.push(1);
    original.push(2);
    original.push(3);

    CircBuffer<int> assigned(3);
    assigned = original;

    TEST_ASSERT_EQUAL(original.getSize(), assigned.getSize());
    TEST_ASSERT_EQUAL(original.getCount(), assigned.getCount());
    TEST_ASSERT_EQUAL(1, assigned[0]);
    TEST_ASSERT_EQUAL(2, assigned[1]);
    TEST_ASSERT_EQUAL(3, assigned[2]);
    local_tearDown();
}

void test_copy_assignment_self_assignment()
{
    local_setUp();
    CircBuffer<int> buffer(5);
    buffer.push(1);
    buffer.push(2);

    buffer = buffer;  // Self-assignment

    TEST_ASSERT_EQUAL(2, buffer.getCount());
    TEST_ASSERT_EQUAL(1, buffer[0]);
    TEST_ASSERT_EQUAL(2, buffer[1]);
    local_tearDown();
}

void test_copy_assignment_different_sizes()
{
    local_setUp();
    CircBuffer<int> small(3);
    small.push(1);
    small.push(2);
    small.push(3);

    CircBuffer<int> large(10);
    large = small;

    TEST_ASSERT_EQUAL(3, large.getSize());
    TEST_ASSERT_EQUAL(3, large.getCount());
    local_tearDown();
}

void test_push_single_element()
{
    local_setUp();
    intBuffer->push(42);
    TEST_ASSERT_EQUAL(1, intBuffer->getCount());
    TEST_ASSERT_EQUAL(42, intBuffer->peek());
    local_tearDown();
}

void test_push_multiple_elements()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);

    TEST_ASSERT_EQUAL(3, intBuffer->getCount());
    TEST_ASSERT_EQUAL(1, (*intBuffer)[0]);
    TEST_ASSERT_EQUAL(2, (*intBuffer)[1]);
    TEST_ASSERT_EQUAL(3, (*intBuffer)[2]);
    local_tearDown();
}

void test_push_to_full_buffer_overwrites_oldest()
{
    local_setUp();
    // Buffer size is 5
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);
    intBuffer->push(4);
    intBuffer->push(5);

    TEST_ASSERT_TRUE(intBuffer->isFull());

    // Push 6th element - should overwrite oldest (1)
    intBuffer->push(6);

    TEST_ASSERT_EQUAL(5, intBuffer->getCount());
    TEST_ASSERT_EQUAL(2, intBuffer->peek());  // Oldest is now 2
    local_tearDown();
}

void test_push_wrap_around()
{
    local_setUp();
    // Fill buffer
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);
    intBuffer->push(4);
    intBuffer->push(5);

    // Overwrite with new elements
    intBuffer->push(6);
    intBuffer->push(7);

    TEST_ASSERT_EQUAL(3, (*intBuffer)[0]);  // Oldest
    TEST_ASSERT_EQUAL(7, (*intBuffer)[4]);  // Newest
    local_tearDown();
}

void test_push_different_types()
{
    local_setUp();
    doubleBuffer->push(3.14);
    floatBuffer->push(2.718f);

    TEST_ASSERT_EQUAL_FLOAT(3.14, doubleBuffer->peek());
    TEST_ASSERT_EQUAL_FLOAT(2.718f, floatBuffer->peek());
    local_tearDown();
}

void test_pop_single_element()
{
    local_setUp();
    intBuffer->push(42);
    int value = intBuffer->pop();

    TEST_ASSERT_EQUAL(42, value);
    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    local_tearDown();
}

void test_pop_multiple_elements_fifo()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);

    TEST_ASSERT_EQUAL(1, intBuffer->pop());
    TEST_ASSERT_EQUAL(2, intBuffer->pop());
    TEST_ASSERT_EQUAL(3, intBuffer->pop());
    local_tearDown();
}

void test_pop_from_empty_buffer_returns_default()
{
    local_setUp();
    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    int value = intBuffer->pop();
    TEST_ASSERT_EQUAL(0, value);  // Default constructor for int
    local_tearDown();
}

void test_pop_reduces_count()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);

    TEST_ASSERT_EQUAL(3, intBuffer->getCount());
    intBuffer->pop();
    TEST_ASSERT_EQUAL(2, intBuffer->getCount());
    intBuffer->pop();
    TEST_ASSERT_EQUAL(1, intBuffer->getCount());
    local_tearDown();
}

void test_pop_after_wraparound()
{
    local_setUp();
    // Fill buffer
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);
    intBuffer->push(4);
    intBuffer->push(5);

    // Cause wraparound
    intBuffer->push(6);
    intBuffer->push(7);

    // Pop should give 3, 4, 5, 6, 7
    TEST_ASSERT_EQUAL(3, intBuffer->pop());
    TEST_ASSERT_EQUAL(4, intBuffer->pop());
    TEST_ASSERT_EQUAL(5, intBuffer->pop());
    TEST_ASSERT_EQUAL(6, intBuffer->pop());
    TEST_ASSERT_EQUAL(7, intBuffer->pop());
    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    local_tearDown();
}

void test_peek_does_not_remove_element()
{
    local_setUp();
    intBuffer->push(42);

    TEST_ASSERT_EQUAL(42, intBuffer->peek());
    TEST_ASSERT_EQUAL(1, intBuffer->getCount());
    TEST_ASSERT_EQUAL(42, intBuffer->peek());  // Can peek multiple times
    local_tearDown();
}

void test_peek_empty_buffer_returns_default()
{
    local_setUp();
    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    int value = intBuffer->peek();
    TEST_ASSERT_EQUAL(0, value);
    local_tearDown();
}

void test_peek_returns_oldest_element()
{
    local_setUp();
    intBuffer->push(10);
    intBuffer->push(20);
    intBuffer->push(30);

    TEST_ASSERT_EQUAL(10, intBuffer->peek());
    local_tearDown();
}

void test_clear_empty_buffer()
{
    local_setUp();
    intBuffer->clear();
    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    TEST_ASSERT_EQUAL(0, intBuffer->getCount());
    local_tearDown();
}

void test_clear_populated_buffer()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);

    intBuffer->clear();

    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    TEST_ASSERT_EQUAL(0, intBuffer->getCount());
    local_tearDown();
}

void test_clear_resets_indices()
{
    local_setUp();
    // Fill and wrap
    for (int i = 0; i < 10; i++)
    {
        intBuffer->push(i);
    }

    intBuffer->clear();

    // Should be able to fill normally again
    intBuffer->push(100);
    intBuffer->push(101);
    TEST_ASSERT_EQUAL(100, intBuffer->peek());
    local_tearDown();
}

void test_isEmpty_on_new_buffer()
{
    local_setUp();
    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    local_tearDown();
}

void test_isEmpty_after_push()
{
    local_setUp();
    intBuffer->push(1);
    TEST_ASSERT_FALSE(intBuffer->isEmpty());
    local_tearDown();
}

void test_isEmpty_after_clear()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->clear();
    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    local_tearDown();
}

void test_isFull_reaches_capacity()
{
    local_setUp();
    // Buffer size is 5
    TEST_ASSERT_FALSE(intBuffer->isFull());

    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);
    intBuffer->push(4);
    TEST_ASSERT_FALSE(intBuffer->isFull());

    intBuffer->push(5);
    TEST_ASSERT_TRUE(intBuffer->isFull());
    local_tearDown();
}

void test_isFull_after_overwrite()
{
    local_setUp();
    // Fill buffer
    for (int i = 0; i < 5; i++)
    {
        intBuffer->push(i);
    }

    // Overwrite
    intBuffer->push(100);

    TEST_ASSERT_TRUE(intBuffer->isFull());
    local_tearDown();
}

void test_isFull_after_pop()
{
    local_setUp();
    // Fill buffer
    for (int i = 0; i < 5; i++)
    {
        intBuffer->push(i);
    }

    TEST_ASSERT_TRUE(intBuffer->isFull());

    intBuffer->pop();
    TEST_ASSERT_FALSE(intBuffer->isFull());
    local_tearDown();
}

void test_indexing_operator_read()
{
    local_setUp();
    intBuffer->push(10);
    intBuffer->push(20);
    intBuffer->push(30);

    TEST_ASSERT_EQUAL(10, (*intBuffer)[0]);
    TEST_ASSERT_EQUAL(20, (*intBuffer)[1]);
    TEST_ASSERT_EQUAL(30, (*intBuffer)[2]);
    local_tearDown();
}

void test_indexing_operator_write()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);

    (*intBuffer)[1] = 100;

    TEST_ASSERT_EQUAL(1, (*intBuffer)[0]);
    TEST_ASSERT_EQUAL(100, (*intBuffer)[1]);
    TEST_ASSERT_EQUAL(3, (*intBuffer)[2]);
    local_tearDown();
}

void test_indexing_operator_after_wraparound()
{
    local_setUp();
    // Fill buffer
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);
    intBuffer->push(4);
    intBuffer->push(5);

    // Wrap
    intBuffer->push(6);
    intBuffer->push(7);

    // Index 0 should be oldest (3)
    TEST_ASSERT_EQUAL(3, (*intBuffer)[0]);
    TEST_ASSERT_EQUAL(4, (*intBuffer)[1]);
    TEST_ASSERT_EQUAL(7, (*intBuffer)[4]);
    local_tearDown();
}

void test_indexing_operator_const()
{
    local_setUp();
    intBuffer->push(10);
    intBuffer->push(20);

    const CircBuffer<int>& constRef = *intBuffer;

    TEST_ASSERT_EQUAL(10, constRef[0]);
    TEST_ASSERT_EQUAL(20, constRef[1]);
    local_tearDown();
}

void test_indexing_iterate_all_elements()
{
    local_setUp();
    for (int i = 0; i < 5; i++)
    {
        intBuffer->push(i * 10);
    }

    for (int i = 0; i < intBuffer->getCount(); i++)
    {
        TEST_ASSERT_EQUAL(i * 10, (*intBuffer)[i]);
    }
    local_tearDown();
}

void test_getCount_empty()
{
    local_setUp();
    TEST_ASSERT_EQUAL(0, intBuffer->getCount());
    local_tearDown();
}

void test_getCount_after_pushes()
{
    local_setUp();
    intBuffer->push(1);
    TEST_ASSERT_EQUAL(1, intBuffer->getCount());

    intBuffer->push(2);
    TEST_ASSERT_EQUAL(2, intBuffer->getCount());

    intBuffer->push(3);
    TEST_ASSERT_EQUAL(3, intBuffer->getCount());
    local_tearDown();
}

void test_getCount_after_pops()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);

    intBuffer->pop();
    TEST_ASSERT_EQUAL(2, intBuffer->getCount());

    intBuffer->pop();
    TEST_ASSERT_EQUAL(1, intBuffer->getCount());
    local_tearDown();
}

void test_getCount_max_is_size()
{
    local_setUp();
    // Fill past capacity
    for (int i = 0; i < 10; i++)
    {
        intBuffer->push(i);
    }

    TEST_ASSERT_EQUAL(5, intBuffer->getCount());
    TEST_ASSERT_EQUAL(5, intBuffer->getSize());
    local_tearDown();
}

void test_getSize_returns_capacity()
{
    local_setUp();
    TEST_ASSERT_EQUAL(5, intBuffer->getSize());
    TEST_ASSERT_EQUAL(5, doubleBuffer->getSize());
    TEST_ASSERT_EQUAL(5, floatBuffer->getSize());
    local_tearDown();
}

void test_getSize_unchanged_by_operations()
{
    local_setUp();
    TEST_ASSERT_EQUAL(5, intBuffer->getSize());

    intBuffer->push(1);
    TEST_ASSERT_EQUAL(5, intBuffer->getSize());

    intBuffer->pop();
    TEST_ASSERT_EQUAL(5, intBuffer->getSize());

    intBuffer->clear();
    TEST_ASSERT_EQUAL(5, intBuffer->getSize());
    local_tearDown();
}

void test_single_element_buffer()
{
    local_setUp();
    CircBuffer<int> buf(1);

    TEST_ASSERT_TRUE(buf.isEmpty());

    buf.push(42);
    TEST_ASSERT_TRUE(buf.isFull());
    TEST_ASSERT_EQUAL(42, buf.peek());

    buf.push(99);  // Overwrite
    TEST_ASSERT_EQUAL(99, buf.peek());
    local_tearDown();
}

void test_alternating_push_pop()
{
    local_setUp();
    intBuffer->push(1);
    TEST_ASSERT_EQUAL(1, intBuffer->pop());

    intBuffer->push(2);
    TEST_ASSERT_EQUAL(2, intBuffer->pop());

    intBuffer->push(3);
    TEST_ASSERT_EQUAL(3, intBuffer->pop());

    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    local_tearDown();
}

void test_fill_empty_fill_pattern()
{
    local_setUp();
    // Fill
    for (int i = 0; i < 5; i++)
    {
        intBuffer->push(i);
    }
    TEST_ASSERT_TRUE(intBuffer->isFull());

    // Empty
    while (!intBuffer->isEmpty())
    {
        intBuffer->pop();
    }
    TEST_ASSERT_TRUE(intBuffer->isEmpty());

    // Fill again
    for (int i = 100; i < 105; i++)
    {
        intBuffer->push(i);
    }
    TEST_ASSERT_TRUE(intBuffer->isFull());
    TEST_ASSERT_EQUAL(100, intBuffer->peek());
    local_tearDown();
}

void test_many_wraparounds()
{
    local_setUp();
    // Cause multiple wraparounds
    for (int i = 0; i < 100; i++)
    {
        intBuffer->push(i);
    }

    // Should have last 5 elements
    TEST_ASSERT_EQUAL(5, intBuffer->getCount());
    TEST_ASSERT_EQUAL(95, (*intBuffer)[0]);
    TEST_ASSERT_EQUAL(99, (*intBuffer)[4]);
    local_tearDown();
}

void test_partial_fill_operations()
{
    local_setUp();
    intBuffer->push(1);
    intBuffer->push(2);

    TEST_ASSERT_EQUAL(1, intBuffer->pop());

    intBuffer->push(3);
    intBuffer->push(4);

    TEST_ASSERT_EQUAL(2, intBuffer->pop());
    TEST_ASSERT_EQUAL(3, intBuffer->pop());
    TEST_ASSERT_EQUAL(4, intBuffer->pop());

    TEST_ASSERT_TRUE(intBuffer->isEmpty());
    local_tearDown();
}

void test_double_precision()
{
    local_setUp();
    doubleBuffer->push(3.141592653589793);
    doubleBuffer->push(2.718281828459045);

    TEST_ASSERT_EQUAL_DOUBLE(3.141592653589793, doubleBuffer->pop());
    TEST_ASSERT_EQUAL_DOUBLE(2.718281828459045, doubleBuffer->pop());
    local_tearDown();
}

void test_negative_values()
{
    local_setUp();
    intBuffer->push(-10);
    intBuffer->push(-20);
    intBuffer->push(-30);

    TEST_ASSERT_EQUAL(-10, (*intBuffer)[0]);
    TEST_ASSERT_EQUAL(-20, (*intBuffer)[1]);
    TEST_ASSERT_EQUAL(-30, (*intBuffer)[2]);
    local_tearDown();
}

void test_zero_values()
{
    local_setUp();
    intBuffer->push(0);
    intBuffer->push(0);
    intBuffer->push(0);

    TEST_ASSERT_EQUAL(0, intBuffer->pop());
    TEST_ASSERT_EQUAL(0, intBuffer->pop());
    TEST_ASSERT_EQUAL(0, intBuffer->pop());
    local_tearDown();
}

void test_large_number_of_operations()
{
    local_setUp();
    // Perform 1000 operations
    for (int i = 0; i < 1000; i++)
    {
        intBuffer->push(i);

        if (i % 3 == 0)
        {
            intBuffer->pop();
        }
    }

    // Buffer should still be valid
    TEST_ASSERT_TRUE(intBuffer->getCount() <= intBuffer->getSize());
    local_tearDown();
}

void test_rapid_fill_and_drain()
{
    local_setUp();
    for (int cycle = 0; cycle < 10; cycle++)
    {
        // Fill
        for (int i = 0; i < 5; i++)
        {
            intBuffer->push(cycle * 10 + i);
        }

        // Drain
        for (int i = 0; i < 5; i++)
        {
            intBuffer->pop();
        }

        TEST_ASSERT_TRUE(intBuffer->isEmpty());
    }
    local_tearDown();
}

void test_use_as_sliding_window()
{
    local_setUp();
    // Simulate sliding window of size 5
    for (int i = 0; i < 10; i++)
    {
        intBuffer->push(i);

        if (intBuffer->getCount() == 5)
        {
            // Calculate sum of window
            int sum = 0;
            for (int j = 0; j < 5; j++)
            {
                sum += (*intBuffer)[j];
            }

            // Verify sliding window sum
            if (i == 4) TEST_ASSERT_EQUAL(10, sum);  // 0+1+2+3+4
            if (i == 5) TEST_ASSERT_EQUAL(15, sum);  // 1+2+3+4+5
        }
    }
    local_tearDown();
}

void test_use_as_queue()
{
    local_setUp();
    // Queue operations: enqueue and dequeue
    intBuffer->push(1);
    intBuffer->push(2);
    intBuffer->push(3);

    TEST_ASSERT_EQUAL(1, intBuffer->pop());  // FIFO

    intBuffer->push(4);
    intBuffer->push(5);

    TEST_ASSERT_EQUAL(2, intBuffer->pop());
    TEST_ASSERT_EQUAL(3, intBuffer->pop());
    TEST_ASSERT_EQUAL(4, intBuffer->pop());
    TEST_ASSERT_EQUAL(5, intBuffer->pop());
    local_tearDown();
}

void run_test_circular_buffer_tests()
{
    suite_setUp();
    RUN_TEST(test_constructor_creates_empty_buffer);
    RUN_TEST(test_constructor_different_sizes);
    RUN_TEST(test_copy_constructor_empty_buffer);
    RUN_TEST(test_copy_constructor_populated_buffer);
    RUN_TEST(test_copy_constructor_full_buffer);
    RUN_TEST(test_copy_constructor_deep_copy);
    RUN_TEST(test_copy_assignment_operator);
    RUN_TEST(test_copy_assignment_self_assignment);
    RUN_TEST(test_copy_assignment_different_sizes);
    RUN_TEST(test_push_single_element);
    RUN_TEST(test_push_multiple_elements);
    RUN_TEST(test_push_to_full_buffer_overwrites_oldest);
    RUN_TEST(test_push_wrap_around);
    RUN_TEST(test_push_different_types);
    RUN_TEST(test_pop_single_element);
    RUN_TEST(test_pop_multiple_elements_fifo);
    RUN_TEST(test_pop_from_empty_buffer_returns_default);
    RUN_TEST(test_pop_reduces_count);
    RUN_TEST(test_pop_after_wraparound);
    RUN_TEST(test_peek_does_not_remove_element);
    RUN_TEST(test_peek_empty_buffer_returns_default);
    RUN_TEST(test_peek_returns_oldest_element);
    RUN_TEST(test_clear_empty_buffer);
    RUN_TEST(test_clear_populated_buffer);
    RUN_TEST(test_clear_resets_indices);
    RUN_TEST(test_isEmpty_on_new_buffer);
    RUN_TEST(test_isEmpty_after_push);
    RUN_TEST(test_isEmpty_after_clear);
    RUN_TEST(test_isFull_reaches_capacity);
    RUN_TEST(test_isFull_after_overwrite);
    RUN_TEST(test_isFull_after_pop);
    RUN_TEST(test_indexing_operator_read);
    RUN_TEST(test_indexing_operator_write);
    RUN_TEST(test_indexing_operator_after_wraparound);
    RUN_TEST(test_indexing_operator_const);
    RUN_TEST(test_indexing_iterate_all_elements);
    RUN_TEST(test_getCount_empty);
    RUN_TEST(test_getCount_after_pushes);
    RUN_TEST(test_getCount_after_pops);
    RUN_TEST(test_getCount_max_is_size);
    RUN_TEST(test_getSize_returns_capacity);
    RUN_TEST(test_getSize_unchanged_by_operations);
    RUN_TEST(test_single_element_buffer);
    RUN_TEST(test_alternating_push_pop);
    RUN_TEST(test_fill_empty_fill_pattern);
    RUN_TEST(test_many_wraparounds);
    RUN_TEST(test_partial_fill_operations);
    RUN_TEST(test_double_precision);
    RUN_TEST(test_negative_values);
    RUN_TEST(test_zero_values);
    RUN_TEST(test_large_number_of_operations);
    RUN_TEST(test_rapid_fill_and_drain);
    RUN_TEST(test_use_as_sliding_window);
    RUN_TEST(test_use_as_queue);
    suite_tearDown();
}

} // namespace test_circular_buffer
