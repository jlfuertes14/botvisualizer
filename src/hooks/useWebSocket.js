import { useState, useCallback, useRef, useEffect } from 'react';

/**
 * Custom hook for WebSocket connection to ESP32 Maze Solver
 * 
 * Expected data formats from ESP32:
 * 
 * Telemetry (every cell):
 * {
 *   "type": "telemetry",
 *   "position": { "x": 2, "y": 3 },
 *   "heading": 90,
 *   "walls": { "front": true, "left": false, "right": true },
 *   "episode": 12,
 *   "step": 45,
 *   "reward": -32,
 *   "isGoal": false
 * }
 * 
 * Episode complete:
 * {
 *   "type": "episode_complete",
 *   "episode": 12,
 *   "totalSteps": 45,
 *   "totalReward": 87,
 *   "solved": true
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
        episode: 0,
        step: 0,
        reward: 0,
        isGoal: false,
        timestamp: Date.now(),
    });

    // New states for visualisation
    const [blockedWalls, setBlockedWalls] = useState({}); // key "x,y,dir" -> true
    const [qTable, setQTable] = useState({}); // key "x,y" -> [qN,qE,qS,qW]


    // Episode history
    const [episodeHistory, setEpisodeHistory] = useState([]);

    // Discovered walls map: key = "x,y,direction" -> value = true/false
    const [discoveredWalls, setDiscoveredWalls] = useState({});

    // Path history for current episode
    const [pathHistory, setPathHistory] = useState([]);

    // Legacy sensor data format (for backward compatibility)
    const [sensorData, setSensorData] = useState({
        yaw: 0,
        accelX: 0,
        accelY: 0,
        accelZ: 0,
        direction: 'stop',
        speed: 0,
        timestamp: Date.now(),
    });

    const wsRef = useRef(null);

    const connect = useCallback((ipAddress, port = 81) => {
        if (wsRef.current) {
            wsRef.current.close();
        }

        setIsConnecting(true);
        setError(null);

        try {
            const wsUrl = `ws://${ipAddress}:${port}`;
            console.log(`Connecting to ${wsUrl}...`);

            const ws = new WebSocket(wsUrl);
            wsRef.current = ws;

            ws.onopen = () => {
                console.log('WebSocket connected!');
                setIsConnected(true);
                setIsConnecting(false);
                setError(null);
            };

            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);

                    if (data.type === 'telemetry') {
                        // Maze telemetry update
                        const newMazeData = {
                            position: data.position || { x: 0, y: 0 },
                            heading: data.heading || 0,
                            walls: data.walls || { front: false, left: false, right: false },
                            episode: data.episode || 0,
                            step: data.step || 0,
                            reward: data.reward || 0,
                            isGoal: data.isGoal || false,
                            timestamp: Date.now(),
                        };
                        setMazeData(newMazeData);

                        // Add to path history
                        setPathHistory(prev => [...prev, { ...data.position, heading: data.heading }]);

                        // Update discovered walls
                        if (data.walls && data.position) {
                            const { x, y } = data.position;
                            const heading = data.heading;
                            setDiscoveredWalls(prev => {
                                const updated = { ...prev };
                                // Map walls based on robot heading
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

                        // Convert to legacy format for backward compatibility
                        setSensorData({
                            yaw: data.heading || 0,
                            accelX: 0,
                            accelY: data.isGoal ? 0 : 0.5,
                            accelZ: 1,
                            direction: data.step > 0 ? 'forward' : 'stop',
                            speed: 0.5,
                            timestamp: Date.now(),
                        });

                    } else if (data.type === 'episode_complete') {
                        // Episode finished
                        setEpisodeHistory(prev => [...prev, {
                            episode: data.episode,
                            steps: data.totalSteps,
                            reward: data.totalReward,
                            solved: data.solved,
                        }]);

                        // Clear path history for next episode
                        setPathHistory([]);

                    } else if (data.type === 'collision') {
                        // ESP32 reports a blocked move; mark the wall as blocked
                        const { position, heading } = data; // heading in degrees
                        const dirs = ['north', 'east', 'south', 'west'];
                        const dirIdx = Math.floor(((heading % 360) + 360) % 360 / 90) % 4;
                        const blockedDir = dirs[dirIdx];
                        const key = `${position.x},${position.y},${blockedDir}`;
                        setBlockedWalls(prev => ({ ...prev, [key]: true }));
                    } else if (data.type === 'q_table') {
                        // Full Q‑table for the current episode
                        setQTable(data.data || {});
                    } else {
                        // Legacy format (backward compatibility)
                        let direction = data.direction || 'stop';
                        let speed = 0;

                        if (data.accelY !== undefined) {
                            const threshold = 0.15;
                            if (data.accelY > threshold) {
                                direction = 'forward';
                                speed = Math.min(data.accelY, 1);
                            } else if (data.accelY < -threshold) {
                                direction = 'backward';
                                speed = Math.max(data.accelY, -1);
                            }
                        }

                        setSensorData({
                            yaw: data.yaw ?? data.heading ?? 0,
                            accelX: data.accelX ?? data.ax ?? 0,
                            accelY: data.accelY ?? data.ay ?? 0,
                            accelZ: data.accelZ ?? data.az ?? 0,
                            direction,
                            speed,
                            timestamp: Date.now(),
                        });
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
    }, []);

    const disconnect = useCallback(() => {
        if (wsRef.current) {
            wsRef.current.close(1000, 'User disconnected');
            wsRef.current = null;
        }
        setIsConnected(false);
        setIsConnecting(false);
        setError(null);
    }, []);

    const resetMazeData = useCallback(() => {
        setMazeData({
            position: { x: 0, y: 0 },
            heading: 0,
            walls: { front: false, left: false, right: false },
            episode: 0,
            step: 0,
            reward: 0,
            isGoal: false,
            timestamp: Date.now(),
        });
        setDiscoveredWalls({});
        setPathHistory([]);
        setEpisodeHistory([]);
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
        episodeHistory,
        connect,
        disconnect,
        setSensorData,
        setMazeData,
        resetMazeData,
    };
}
