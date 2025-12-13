import { useState, useEffect, useRef, useCallback } from 'react';

/**
 * Dashboard Component
 * - Connection panel (IP input, connect/disconnect buttons)
 * - Sensor data display (yaw compass, ultrasonic sensors, wall detection)
 * - Demo mode button
 */
export function Dashboard({
    sensorData,
    isConnected,
    isConnecting,
    error,
    onConnect,
    onDisconnect,
    onDemoData,
    onStartMaze,
    onStopMaze,
    mazeConfig
}) {
    const [ipAddress, setIpAddress] = useState('botvisualizer-production.up.railway.app');
    const [port, setPort] = useState('443');
    const [isDemoMode, setIsDemoMode] = useState(false);
    const demoIntervalRef = useRef(null);

    // Demo mode - simulates sensor data
    const startDemo = useCallback(() => {
        setIsDemoMode(true);
        let angle = 0;
        let direction = 'forward';
        let directionTimer = 0;

        demoIntervalRef.current = setInterval(() => {
            // Rotate the yaw
            angle = (angle + 2) % 360;

            // Change direction every ~3 seconds
            directionTimer++;
            if (directionTimer > 60) {
                directionTimer = 0;
                if (direction === 'forward') direction = 'stop';
                else if (direction === 'stop') direction = 'backward';
                else direction = 'forward';
            }

            onDemoData({
                yaw: angle,
                direction,
                speed: direction === 'forward' ? 1 : direction === 'backward' ? -1 : 0,
                timestamp: Date.now(),
            });
        }, 50);
    }, [onDemoData]);

    const stopDemo = useCallback(() => {
        setIsDemoMode(false);
        if (demoIntervalRef.current) {
            clearInterval(demoIntervalRef.current);
            demoIntervalRef.current = null;
        }
        onDemoData({
            yaw: 0,
            direction: 'stop',
            speed: 0,
            timestamp: Date.now(),
        });
    }, [onDemoData]);

    // Cleanup on unmount
    useEffect(() => {
        return () => {
            if (demoIntervalRef.current) {
                clearInterval(demoIntervalRef.current);
            }
        };
    }, []);

    const handleConnect = () => {
        if (isDemoMode) stopDemo();
        onConnect(ipAddress, parseInt(port));
    };

    const handleToggleDemo = () => {
        if (isConnected) onDisconnect();
        if (isDemoMode) {
            stopDemo();
        } else {
            startDemo();
        }
    };

    // Connection status
    const connectionStatus = isDemoMode ? 'demo' :
        isConnected ? 'connected' :
            isConnecting ? 'connecting' : 'disconnected';

    return (
        <div className="flex flex-col gap-4 p-4 h-full overflow-y-auto">
            {/* Header */}
            <div className="text-center mb-2">
                <h2 className="text-xl font-bold text-cyber-cyan">Control Panel</h2>
            </div>

            {/* Connection Status Card */}
            <div className="glass-card p-4">
                <div className="flex items-center justify-between mb-3">
                    <span className="text-sm text-slate-400">Status</span>
                    <div className="flex items-center gap-2">
                        <div className={`status-dot ${connectionStatus === 'demo' ? 'connected' : connectionStatus}`}></div>
                        <span className={`text-sm font-medium ${connectionStatus === 'connected' || connectionStatus === 'demo' ? 'text-cyber-green' :
                            connectionStatus === 'connecting' ? 'text-yellow-400' : 'text-cyber-red'
                            }`}>
                            {connectionStatus === 'demo' ? 'Demo Mode' :
                                connectionStatus === 'connected' ? 'Connected' :
                                    connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
                        </span>
                    </div>
                </div>

                {error && (
                    <div className="text-cyber-red text-sm mb-3 p-2 bg-red-900/20 rounded">
                        ⚠️ {error}
                    </div>
                )}

                {/* IP Address Input */}
                <div className="space-y-3">
                    <div>
                        <label className="text-xs text-slate-400 block mb-1">ESP32 IP Address</label>
                        <input
                            type="text"
                            value={ipAddress}
                            onChange={(e) => setIpAddress(e.target.value)}
                            placeholder="your-app.up.railway.app"
                            className="w-full"
                            disabled={isConnected || isConnecting}
                        />
                    </div>

                    <div>
                        <label className="text-xs text-slate-400 block mb-1">WebSocket Port</label>
                        <input
                            type="text"
                            value={port}
                            onChange={(e) => setPort(e.target.value)}
                            placeholder="81"
                            className="w-full"
                            disabled={isConnected || isConnecting}
                        />
                    </div>

                    {/* Connection Buttons */}
                    <div className="flex gap-2">
                        {!isConnected && !isConnecting ? (
                            <button
                                onClick={handleConnect}
                                className="cyber-btn flex-1"
                                disabled={isDemoMode}
                            >
                                Connect
                            </button>
                        ) : (
                            <button
                                onClick={onDisconnect}
                                className="cyber-btn disconnect flex-1"
                            >
                                Disconnect
                            </button>
                        )}
                    </div>

                    {/* Demo Mode Button */}
                    <button
                        onClick={handleToggleDemo}
                        className={`cyber-btn demo w-full ${isDemoMode ? 'opacity-80' : ''}`}
                    >
                        {isDemoMode ? '⏹ Stop Demo' : '▶ Demo Mode'}
                    </button>
                </div>
            </div>

            {/* Yaw Display Card */}
            <div className="glass-card p-4">
                <h3 className="text-sm text-slate-400 mb-3 text-center">Yaw / Heading</h3>
                <div className="flex flex-col items-center">
                    <CompassDisplay yaw={sensorData.yaw || 0} />
                    <div className="mt-3 text-2xl font-bold text-cyber-cyan">
                        {(sensorData.yaw || 0).toFixed(1)}°
                    </div>
                </div>
            </div>

            {/* Ultrasonic Sensors Card */}
            <div className="glass-card p-4">
                <h3 className="text-sm text-slate-400 mb-3 text-center">Ultrasonic Sensors</h3>
                <div className="space-y-3">
                    <SensorBar label="Front" value={sensorData.front} hasWall={sensorData.wallFront} />
                    <SensorBar label="Left" value={sensorData.left} hasWall={sensorData.wallLeft} />
                    <SensorBar label="Right" value={sensorData.right} hasWall={sensorData.wallRight} />
                </div>
            </div>

            {/* Robot State Card */}
            <div className="glass-card p-4">
                <h3 className="text-sm text-slate-400 mb-3 text-center">Robot State</h3>
                <div className="flex flex-col items-center">
                    <RobotStateDisplay state={sensorData.state} />
                    <div className="mt-3 text-sm text-slate-300">
                        Position: ({sensorData.x ?? '-'}, {sensorData.y ?? '-'})
                    </div>
                    <div className="text-xs text-slate-500">
                        Heading: {['N', 'E', 'S', 'W'][sensorData.heading] || '-'}
                    </div>
                </div>
            </div>

            {/* Maze Control Card */}
            <div className="glass-card p-4">
                <h3 className="text-sm text-slate-400 mb-3 text-center">Maze Control</h3>
                <div className="space-y-2">
                    <button
                        onClick={onStartMaze}
                        className="cyber-btn w-full"
                        disabled={!isConnected || sensorData.state === 'moving' || sensorData.state === 'thinking'}
                    >
                        🚀 Start Maze
                    </button>
                    <button
                        onClick={onStopMaze}
                        className="cyber-btn disconnect w-full"
                        disabled={!isConnected || sensorData.state === 'idle'}
                    >
                        ⏹ Stop
                    </button>
                </div>
                {mazeConfig && (
                    <div className="mt-3 text-xs text-slate-500">
                        <div>Start: ({mazeConfig.start?.x}, {mazeConfig.start?.y})</div>
                        <div>Goal: ({mazeConfig.goal?.x}, {mazeConfig.goal?.y})</div>
                    </div>
                )}
            </div>

            {/* Live Data Stream Card */}
            <div className="glass-card p-4">
                <h3 className="text-sm text-slate-400 mb-3">Live Sensor Data</h3>
                <div className="font-mono text-xs bg-slate-900/50 rounded p-2 overflow-x-auto max-h-32">
                    <pre className="text-cyber-cyan">
                        {JSON.stringify({
                            front: sensorData.front?.toFixed(1) + 'cm',
                            left: sensorData.left?.toFixed(1) + 'cm',
                            right: sensorData.right?.toFixed(1) + 'cm',
                            yaw: (sensorData.yaw || 0).toFixed(1) + '°',
                            state: sensorData.state,
                            pos: `(${sensorData.x}, ${sensorData.y})`
                        }, null, 2)}
                    </pre>
                </div>
            </div>
        </div>
    );
}

/**
 * Compass display showing yaw angle
 */
function CompassDisplay({ yaw }) {
    const needleStyle = {
        transform: `translate(-50%, 0) rotate(${yaw}deg)`,
        transformOrigin: 'bottom center',
    };

    return (
        <div className="compass">
            {/* Direction Labels */}
            <span className="absolute top-1 left-1/2 -translate-x-1/2 text-xs font-bold text-cyber-red">N</span>
            <span className="absolute bottom-1 left-1/2 -translate-x-1/2 text-xs text-slate-500">S</span>
            <span className="absolute left-1 top-1/2 -translate-y-1/2 text-xs text-slate-500">W</span>
            <span className="absolute right-1 top-1/2 -translate-y-1/2 text-xs text-slate-500">E</span>

            {/* Needle */}
            <div
                className="compass-needle"
                style={needleStyle}
            />
        </div>
    );
}

/**
 * Direction indicator with arrow
 */
function DirectionDisplay({ direction }) {
    const directionConfig = {
        forward: { icon: '↑', label: 'Forward', color: 'text-cyber-green', bg: 'bg-green-900/30' },
        backward: { icon: '↓', label: 'Backward', color: 'text-cyber-red', bg: 'bg-red-900/30' },
        stop: { icon: '■', label: 'Stopped', color: 'text-slate-400', bg: 'bg-slate-800/50' },
    };

    const config = directionConfig[direction] || directionConfig.stop;

    return (
        <div className={`flex flex-col items-center p-4 rounded-lg ${config.bg}`}>
            <span className={`text-4xl ${config.color} direction-arrow ${direction}`}>
                {config.icon}
            </span>
            <span className={`text-sm mt-2 font-medium ${config.color}`}>
                {config.label}
            </span>
        </div>
    );
}

/**
 * Ultrasonic sensor bar display
 */
function SensorBar({ label, value, hasWall }) {
    const distance = value || 999;
    const maxDistance = 50; // cm for visual scale
    const percentage = Math.min((distance / maxDistance) * 100, 100);

    // Color based on distance
    const getBarColor = () => {
        if (hasWall || distance < 15) return 'bg-cyber-red';
        if (distance < 25) return 'bg-yellow-500';
        return 'bg-cyber-green';
    };

    return (
        <div className="flex items-center gap-2">
            <span className="text-xs text-slate-400 w-12">{label}</span>
            <div className="flex-1 h-3 bg-slate-800 rounded overflow-hidden">
                <div
                    className={`h-full transition-all duration-200 ${getBarColor()}`}
                    style={{ width: `${100 - percentage}%` }}
                />
            </div>
            <span className={`text-xs font-mono w-14 text-right ${hasWall ? 'text-cyber-red' : 'text-slate-300'}`}>
                {distance < 999 ? `${distance.toFixed(1)}cm` : '---'}
            </span>
            {hasWall && <span className="text-cyber-red text-xs">⚠</span>}
        </div>
    );
}

/**
 * Robot state display with icons
 */
function RobotStateDisplay({ state }) {
    const stateConfig = {
        idle: { icon: '💤', label: 'Idle', color: 'text-slate-400', bg: 'bg-slate-800/50' },
        thinking: { icon: '🤔', label: 'Thinking', color: 'text-yellow-400', bg: 'bg-yellow-900/30' },
        moving: { icon: '🏃', label: 'Moving', color: 'text-cyber-cyan', bg: 'bg-cyan-900/30' },
        goal: { icon: '🎉', label: 'Goal!', color: 'text-cyber-green', bg: 'bg-green-900/30' },
    };

    const config = stateConfig[state] || stateConfig.idle;

    return (
        <div className={`flex items-center gap-2 px-4 py-2 rounded-lg ${config.bg}`}>
            <span className="text-2xl">{config.icon}</span>
            <span className={`font-medium ${config.color}`}>{config.label}</span>
        </div>
    );
}

export default Dashboard;

