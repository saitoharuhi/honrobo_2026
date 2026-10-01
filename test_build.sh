#!/bin/bash
while true; do
  if grep -q "Summary" /home/haru/.gemini/antigravity/brain/9a1ae9e5-09a3-45c1-a04c-ed5a58b4484f/.system_generated/tasks/task-8121.log; then
    cat /home/haru/.gemini/antigravity/brain/9a1ae9e5-09a3-45c1-a04c-ed5a58b4484f/.system_generated/tasks/task-8121.log
    break
  fi
  sleep 1
done
