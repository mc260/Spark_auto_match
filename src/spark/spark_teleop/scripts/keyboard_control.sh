#!/bin/bash

# Start keyboard teleop in a new terminal when possible.
# If no GUI terminal is available, fall back to tmux. If neither is available,
# run the node in background and write logs to /tmp.

CMD='rosrun spark_teleop spark_teleop_node 0.14 0.5'

if command -v gnome-terminal >/dev/null 2>&1 && [ -n "$DISPLAY" ]; then
	gnome-terminal --title="spark_control" --geometry 34x10+63+305 -- bash -lc "$CMD; exec bash"
else
	if command -v tmux >/dev/null 2>&1; then
		# create a detached tmux session named spark_control
		tmux new-session -d -s spark_control "$CMD; bash"
		echo "Started keyboard control in tmux session 'spark_control'. Attach with: tmux attach -t spark_control"
	else
		# no terminal multiplexer available, run in background and log output
		nohup $CMD > /tmp/spark_teleop_node.log 2>&1 &
		echo "Started keyboard control in background, logs: /tmp/spark_teleop_node.log"
	fi
fi

