# Balance Board Project

## Important Links

- https://www.waveshare.com/wiki/ESP32-S3-Matrix

## Testing

Tests are located in the `test` directory under each component. Currently, there are tests for:

- Component: utilities
    - QueuedNotificationService unit tests
    - Misc utility function tests

### Running Tests

The project uses the ESP-IDF Unit Test App for running tests. To run:

1. Open an ESP-IDF shell (run `export.sh` or `export.bat`)

2. Build the entire project with tests enabled at the top-level:
   ```bash
   idf.py -DTEST_MODE=ON build
   ```

3. Flash the firmware (including tests) to the board and monitor the output:
   ```bash
   idf.py flash monitor
   ```

4. The test results will automatically be displayed in the serial monitor output showing which tests passed or failed.

### Notes
- If you want to clean the build directory before running the tests, you can use:
  ```bash
  idf.py fullclean
  ```
  
- To run or test specific components instead of the whole project, navigate to that component's directory and run:
  ```bash
  idf.py -DTEST_MODE=ON build flash monitor
  ```
