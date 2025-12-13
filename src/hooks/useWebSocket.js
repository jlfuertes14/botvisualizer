import { useState, useCallback, useRef, useEffect } from 'react';

/**
 * Custom hook for WebSocket connection to ESP32 Wall Follower
 * 
 * Expected data formats from ESP32:
 * 
 * Movement telemetry:
 * {
 *   "type": "movement",
 *   "action": "forward" | "turn_left" | "turn_right" | "u_turn",
 *   "heading": 90.5,
 *   "run": 1,
 *   "move": 45,
 *   "position": { "x": 2, "y": 3 },
 *   "sensors": { "front": 12.5, "left": 8.2, "right": 15.0 },
 *   "walls": { "front": true, "left": false, "right": false }
 * }
 */
export function useWebSocket() {
    const [isConnected, setIsConnected] = useState(false);
    const [isConnecting, setIsConnecting] = useState(false);
    const [error, setError] = useState(null);

    // Maze-specific data
    const [mazeData, setMazeData] = useState({
        position: { x: 0, y: 0 },
        heading: 0,
        walls: { front: false, left: false, right: false },
        run: 0,
        move: 0,
        action: '',
        isGoal: false,
        timestamp: Date.now(),
    });

    // Movement log - tracks all robot movements
    const [movementLog, setMovementLog] = useState([]);

    // Optimized path - computed from movement log
    const [optimizedPath, setOptimizedPath] = useState([]);

    // Run history (formerly episode history)
    const [runHistory, setRunHistory] = useState([]);

    // Robot EEPROM path status
    const [robotPathStatus, setRobotPathStatus] = useState({
        hasSavedPath: false,
        pathLength: 0
    });

    // Discovered walls map: key = "x,y,direction" -> value = true/false
    const [discoveredWalls, setDiscoveredWalls] = useState({});

    // Path history for current run
    const [pathHistory, setPathHistory] = useState([]);

    // Legacy sensor data format (for backward compatibility)
    const [sensorData, setSensorData] = useState({
        yaw: 0,
        front: 0,
        left: 0,
        right: 0,
        wallFront: false,
        wallLeft: false,
        wallRight: false,
        direction: 'stop',
        speed: 0,
        timestamp: Date.now(),
    });

    const wsRef = useRef(null);

    // Path optimizer function
    const optimizePath = useCallback((movements) => {
        if (movements.length < 2) return movements;

        const optimized = [];
        let i = 0;

        while (i < movements.length) {
            const current = movements[i];
            const next = movements[i + 1];

            // Skip opposite turns that cancel out
            if (next) {
                if ((current.action === 'turn_left' && next.action === 'turn_right') ||
                    (current.action === 'turn_right' && next.action === 'turn_left')) {
                    i += 2; // Skip both
                    continue;
                }
            }

            // Skip u_turn followed by u_turn (360 = no turn)
            if (next && current.action === 'u_turn' && next.action === 'u_turn') {
                i += 2;
                continue;
            }

            optimized.push(current);
            i++;
        }

        return optimized;
    }, []);

    const connect = useCallback((ipAddress, port = 81) => {
        if (wsRef.current) {
            wsRef.current.close();
        }

        setIsConnecting(true);
        setError(null);

        try {
            // Use wss:// for Railway (port 443), ws:// for local
            const protocol = port === '443' || port === 443 ? 'wss' : 'ws';
            const wsUrl = port === '443' || port === 443
                ? `${protocol}://${ipAddress}`
                : `${protocol}://${ipAddress}:${port}`;
            console.log(`Connecting to ${wsUrl}...`);

            const ws = new WebSocket(wsUrl);
            wsRef.current = ws;

            ws.onopen = () => {
                console.log('WebSocket connected to bridge server!');
                setIsConnected(true);
                setIsConnecting(false);
                setError(null);

                // Send identification to bridge server
                ws.send(JSON.stringify({
                    type: 'identify',
                    client: 'browser'
                }));
                console.log('Sent browser identification to bridge');
            };

            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);

                    // Handle ESP32 status from bridge
                    if (data.type === 'esp32_status') {
                        console.log('ESP32 status:', data.connected ? 'connected' : 'disconnected');
                        return;
                    }

                    // Handle path status from robot
                    if (data.type === 'path_status') {
                        console.log('Robot path status:', data);
                        setRobotPathStatus({
                            hasSavedPath: data.hasSavedPath,
                            pathLength: data.pathLength || 0
                        });
                        return;
                    }

                    // Handle path saved confirmation
                    if (data.type === 'path_saved') {
                        console.log('Path saved to robot:', data.length, 'moves');
                        setRobotPathStatus({
                            hasSavedPath: true,
                            pathLength: data.length
                        });
                        return;
                    }

                    // Handle path cleared confirmation
                    if (data.type === 'path_cleared') {
                        console.log('Path cleared from robot');
                        setRobotPathStatus({
                            hasSavedPath: false,
                            pathLength: 0
                        });
                        return;
                    }

                    // Handle sensor telemetry (periodic updates from ESP32)
                    if (data.type === 'sensors') {
                        setSensorData(prev => ({
                            ...prev,
                            front: data.front,
                            left: data.left,
                            right: data.right,
                            yaw: data.yaw ?? prev.yaw,
                            x: data.x,
                            y: data.y,
                            heading: data.heading,
                            state: data.state,
                            wallFront: data.wallFront,
                            wallLeft: data.wallLeft,
                            wallRight: data.wallRight,
                            timestamp: Date.now(),
                        }));
                        return;
                    }

                    // Handle movement telemetry from wall follower
                    if (data.type === 'movement') {
                        const movement = {
                            action: data.action,
                            heading: data.heading || 0,
                            position: data.position || { x: 0, y: 0 },
                            sensors: data.sensors || { front: 0, left: 0, right: 0 },
                            walls: data.walls || { front: false, left: false, right: false },
                            run: data.run || 1,
                            move: data.move || 0,
                            timestamp: Date.now(),
                        };

                        // Add to movement log
                        setMovementLog(prev => {
                            const updated = [...prev, movement];
                            // Compute optimized path
                            setOptimizedPath(optimizePath(updated));
                            return updated;
                        });

                        // Update maze data
                        setMazeData({
                            position: movement.position,
                            heading: movement.heading,
                            walls: movement.walls,
                            run: movement.run,
                            move: movement.move,
                            action: movement.action,
                            isGoal: data.action === 'goal_reached',
                            timestamp: Date.now(),
                        });

                        // Add to path history
                        if (movement.position) {
                            setPathHistory(prev => [...prev, {
                                ...movement.position,
                                heading: movement.heading
                            }]);
                        }

                        // Update discovered walls
                        if (movement.walls && movement.position) {
                            const { x, y } = movement.position;
                            const heading = movement.heading;
                            setDiscoveredWalls(prev => {
                                const updated = { ...prev };
                                const headingDir = Math.floor(((heading % 360) + 360) / 90) % 4;
                                const dirs = ['north', 'east', 'south', 'west'];

                                if (movement.walls.front) {
                                    updated[`${x},${y},${dirs[headingDir]}`] = true;
                                }
                                if (movement.walls.left) {
                                    updated[`${x},${y},${dirs[(headingDir + 3) % 4]}`] = true;
                                }
                                if (movement.walls.right) {
                                    updated[`${x},${y},${dirs[(headingDir + 1) % 4]}`] = true;
                                }
                                return updated;
                            });
                        }

                        // Update sensor data for backward compatibility
                        setSensorData(prev => ({
                            ...prev,
                            yaw: movement.heading,
                            front: movement.sensors.front,
                            left: movement.sensors.left,
                            right: movement.sensors.right,
                            wallFront: movement.walls.front,
                            wallLeft: movement.walls.left,
                            wallRight: movement.walls.right,
                            direction: movement.action === 'forward' ? 'forward' : 'stop',
                            timestamp: Date.now(),
                        }));

                        return;
                    }

                    // Handle run complete (formerly episode complete)
                    if (data.type === 'run_complete' || data.type === 'episode_complete') {
                        setRunHistory(prev => [...prev, {
                            run: data.run || data.episode,
                            moves: data.totalMoves || data.totalSteps,
                            optimizedMoves: optimizedPath.length,
                            solved: data.solved,
                        }]);

                        // Clear for next run
                        setMovementLog([]);
                        setOptimizedPath([]);
                        setPathHistory([]);
                        return;
                    }

                    // Handle telemetry (legacy format)
                    if (data.type === 'telemetry') {
                        const newMazeData = {
                            position: data.position || { x: 0, y: 0 },
                            heading: data.heading || 0,
                            walls: data.walls || { front: false, left: false, right: false },
                            run: data.episode || 0,
                            move: data.step || 0,
                            action: data.action || '',
                            isGoal: data.isGoal || false,
                            timestamp: Date.now(),
                        };
                        setMazeData(newMazeData);

                        if (data.position) {
                            setPathHistory(prev => [...prev, {
                                ...data.position,
                                heading: data.heading
                            }]);
                        }

                        // Update discovered walls
                        if (data.walls && data.position) {
                            const { x, y } = data.position;
                            const heading = data.heading;
                            setDiscoveredWalls(prev => {
                                const updated = { ...prev };
                                const headingDir = Math.floor(heading / 90) % 4;
                                const dirs = ['north', 'east', 'south', 'west'];

                                if (data.walls.front) {
                                    updated[`${x},${y},${dirs[headingDir]}`] = true;
                                }
                                if (data.walls.left) {
                                    updated[`${x},${y},${dirs[(headingDir + 3) % 4]}`] = true;
                                }
                                if (data.walls.right) {
                                    updated[`${x},${y},${dirs[(headingDir + 1) % 4]}`] = true;
                                }
                                return updated;
                            });
                        }
                    }
                } catch (e) {
                    console.error('Failed to parse WebSocket data:', e);
                }
            };

            ws.onerror = (event) => {
                console.error('WebSocket error:', event);
                setError('Connection error');
                setIsConnecting(false);
            };

            ws.onclose = (event) => {
                console.log('WebSocket closed:', event.code, event.reason);
                setIsConnected(false);
                setIsConnecting(false);
                if (event.code !== 1000) setError('Connection lost');
            };

        } catch (e) {
            console.error('Failed to create WebSocket:', e);
            setError(e.message);
            setIsConnecting(false);
        }
    }, [optimizePath]);

    const disconnect = useCallback(() => {
        if (wsRef.current) {
            wsRef.current.close(1000, 'User disconnected');
            wsRef.current = null;
        }
        setIsConnected(false);
        setIsConnecting(false);
        setError(null);
    }, []);

    // Send command to ESP32
    const sendCommand = useCallback((command) => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            const msg = JSON.stringify({ command });
            wsRef.current.send(msg);
            console.log('Sent command:', command);
            return true;
        } else {
            console.warn('Cannot send command - WebSocket not connected');
            return false;
        }
    }, []);

    // Send maze start command with start/goal positions
    const sendMazeStart = useCallback((startPos, goalPos) => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            const msg = JSON.stringify({
                type: 'start',
                start: startPos,
                goal: goalPos
            });
            wsRef.current.send(msg);
            console.log('Sent start command:', msg);
            return true;
        } else {
            console.warn('Cannot send start - WebSocket not connected');
            return false;
        }
    }, []);

    // Send stop command
    const sendStop = useCallback(() => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            const msg = JSON.stringify({ type: 'stop' });
            wsRef.current.send(msg);
            console.log('Sent stop command');
            return true;
        } else {
            console.warn('Cannot send stop - WebSocket not connected');
            return false;
        }
    }, []);

    // Send optimized path to robot for EEPROM storage
    const sendOptimizedPath = useCallback((path) => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            // Convert action names to numeric codes
            const actionCodes = {
                'forward': 0,
                'turn_left': 1,
                'turn_right': 2,
                'u_turn': 3
            };

            const pathArray = path.map(move => {
                const action = move.action || move;
                return actionCodes[action] ?? 0;
            });

            const msg = JSON.stringify({
                command: 'save_path',
                path: pathArray
            });
            wsRef.current.send(msg);
            console.log('Sent path to robot:', pathArray.length, 'moves');
            return true;
        } else {
            console.warn('Cannot send path - WebSocket not connected');
            return false;
        }
    }, []);

    // Clear robot's saved path from EEPROM
    const clearRobotPath = useCallback(() => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            const msg = JSON.stringify({ command: 'clear_path' });
            wsRef.current.send(msg);
            console.log('Sent clear path command');
            return true;
        } else {
            console.warn('Cannot clear path - WebSocket not connected');
            return false;
        }
    }, []);

    const resetMazeData = useCallback(() => {
        setMazeData({
            position: { x: 0, y: 0 },
            heading: 0,
            walls: { front: false, left: false, right: false },
            run: 0,
            move: 0,
            action: '',
            isGoal: false,
            timestamp: Date.now(),
        });
        setDiscoveredWalls({});
        setPathHistory([]);
        setMovementLog([]);
        setOptimizedPath([]);
        setRunHistory([]);
    }, []);

    useEffect(() => {
        return () => disconnect();
    }, [disconnect]);

    return {
        isConnected,
        isConnecting,
        error,
        sensorData,
        mazeData,
        discoveredWalls,
        pathHistory,
        movementLog,
        optimizedPath,
        runHistory,
        connect,
        disconnect,
        sendCommand,
        sendMazeStart,
        sendStop,
        setSensorData,
        setMazeData,
        resetMazeData,
        robotPathStatus,
        sendOptimizedPath,
        clearRobotPath,
    };
}
