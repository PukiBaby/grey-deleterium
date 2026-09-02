#include "main.h"

#include "blaberotatos-lib/api/macros.hpp"

#include <atomic>

void macro_class::start(std::function<void()> macro_routine, std::function<void()> reset_function)
{
    if (macro_task) return; // Doesn't create another task because the task already exists

    macro_task.emplace
    (
        // the [] is a lambda's capture list
        [this,
        macro_routine,
        reset_function]()
        {
            macro_worker(macro_routine, reset_function);
        }             
    );
}

void macro_class::macro_worker(std::function<void()> macro_routine, std::function<void()> reset_function)
{
    while (true) // Makes the macro_task reusable
    {
        // Using cooperative cancellation rather than just killing the task
        pros::Task::notify_take(true, // Zeros the notification counter when the task wakes up
                                TIMEOUT_MAX);
        macro_routine();
        reset_function();
        macro_running = false;
    }
}

void macro_class::run()
{
    macro_running = true;
    if (macro_task) macro_task->notify(); // Need to check what -> is
}