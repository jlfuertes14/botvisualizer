import { useState, useEffect, useRef, useCallback } from 'react';
import { MazePatternGenerator } from './MazePatternGenerator';

const IconCar = ({ className }) => (
    <svg className={className} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M19 17h2c.6 0 1-.4 1-1v-3c0-.9-.7-1.7-1.5-1.9C18.7 10.6 16 10 16 10s-1.3-1.4-2.2-2.3c-.5-.4-1.1-.7-1.8-.7H5c-.6 0-1.1.4-1.4.9l-1.4 2.9A3.7 3.7 0 0 0 2 12v4c0 .6.4 1 1 1h2" /><circle cx="7" cy="17" r="2" /><circle cx="17" cy="17" r="2" /><path d="M5 17h9" /></svg>
);

const IconMaze = ({ className }) => (
    <svg className={className} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
        <rect x="3" y="3" width="18" height="18" rx="2" />
        <path d="M3 9h6v6H3" /><path d="M15 3v6h6" /><path d="M9 15h6v6" /><path d="M15 9h6" />
    </svg>
);

const Gauge = ({ value, label, min = 0, max = 100, color = "#06b6d4", unit = "" }) => {
    const radius = 30;
    const circumference = 2 * Math.PI * radius;
    const offset = circumference - ((value - min) / (max - min)) * circumference;

    return (
        <div className="flex flex-col items-center">
            <div className="relative w-24 h-24 flex items-center justify-center">
                <svg className="w-full h-full rotate-[-90deg]">
                    <circle cx="48" cy="48" r={radius} stroke="#1e293b" strokeWidth="6" fill="transparent" />
                    <circle
                        cx="48" cy="48" r={radius}
                        stroke={color}
                        strokeWidth="6"
                        fill="transparent"
                        strokeDasharray={circumference}
                        strokeDashoffset={offset}
                        strokeLinecap="round"
                        className="transition-all duration-300 ease-out"
                    />
                </svg>
                <div className="absolute inset-0 flex flex-col items-center justify-center">
                    <span className="text-xl font-bold font-mono text-white">
                        {Math.abs(value).toFixed(0)}{unit}
                    </span>
                </div>
            </div>
            <span className="text-[10px] uppercase tracking-wider font-bold text-slate-500 mt-1">{label}</span>
        </div>
    );
};

export function Sidebar({
    sensorData,
    mazeData,
    episodeHistory = [],
    isConnected,
    isConnecting,
    error,
    onConnect,
    onDisconnect,
    onDemoData,
    onDemoMazeData,
    onResetMaze,
    cameraMode,
    onCameraChange,
    mazeMode,
    onMazeModeChange,
    startCell,
    goalCell,
    // Lifted maze generator state
    startCorner,
    goalCorner,
    mazeWalls,
    onStartCornerChange,
    onGoalCornerChange,
    onMazeWallsChange,
    onMazeChange
}) {
    const [ipAddress, setIpAddress] = useState('192.168.1.100');
    const [port, setPort] = useState('81');
    const [isDemoMode, setIsDemoMode] = useState(false);
    const [activeTab, setActiveTab] = useState('controls');
    const demoIntervalRef = useRef(null);
    const mazeDemoRef = useRef({ x: 0, y: 0, heading: 0, episode: 1, step: 0 });

    // Maze demo simulation
    const startMazeDemo = useCallback(() => {
        setIsDemoMode(true);
        const demo = mazeDemoRef.current;
        // Start at the configured start cell
        demo.x = startCell?.x ?? 0;
        demo.y = startCell?.y ?? 0;
        demo.heading = 0;
        demo.episode = 1;
        demo.step = 0;

        demoIntervalRef.current = setInterval(() => {
            demo.step++;

            // Random movement simulation
            const rand = Math.random();
            if (rand < 0.4) {
                // Move forward
                if (demo.heading === 0) demo.y = Math.max(0, demo.y - 1);
                else if (demo.heading === 90) demo.x = Math.min(4, demo.x + 1);
                else if (demo.heading === 180) demo.y = Math.min(4, demo.y + 1);
                else demo.x = Math.max(0, demo.x - 1);
            } else if (rand < 0.7) {
                demo.heading = (demo.heading + 90) % 360;
            } else {
                demo.heading = (demo.heading - 90 + 360) % 360;
            }

            // Check if robot reached the configured goal cell
            const isGoal = demo.x === (goalCell?.x ?? 4) && demo.y === (goalCell?.y ?? 4);

            onDemoMazeData?.({
                position: { x: demo.x, y: demo.y },
                heading: demo.heading,
                walls: { front: Math.random() > 0.5, left: Math.random() > 0.7, right: Math.random() > 0.7 },
                episode: demo.episode,
                step: demo.step,
                reward: isGoal ? 100 : -1,
                isGoal,
                timestamp: Date.now()
            });

            // Also update legacy sensor data
            onDemoData?.({
                yaw: demo.heading,
                accelX: 0,
                accelY: 0.3,
                accelZ: 1,
                direction: 'forward',
                speed: 0.3,
                timestamp: Date.now()
            });

            if (isGoal) {
                demo.episode++;
                demo.x = startCell?.x ?? 0;
                demo.y = startCell?.y ?? 0;
                demo.step = 0;
            }
        }, 800);
    }, [onDemoData, onDemoMazeData, startCell, goalCell]);

    const stopDemo = useCallback(() => {
        setIsDemoMode(false);
        if (demoIntervalRef.current) clearInterval(demoIntervalRef.current);
        onDemoData?.({ yaw: 0, accelX: 0, accelY: 0, accelZ: 1, direction: 'stop', speed: 0, timestamp: Date.now() });
        onDemoMazeData?.({
            position: { x: 0, y: 0 },
            heading: 0,
            walls: { front: false, left: false, right: false },
            episode: 0,
            step: 0,
            reward: 0,
            isGoal: false,
            timestamp: Date.now()
        });
    }, [onDemoData, onDemoMazeData]);

    useEffect(() => { return () => { if (demoIntervalRef.current) clearInterval(demoIntervalRef.current); }; }, []);

    const handleConnect = () => { if (isDemoMode) stopDemo(); onConnect(ipAddress, parseInt(port)); };

    const borderGlow = "border-[#1e293b] focus:border-[#06b6d4] focus:shadow-[0_0_15px_rgba(6,182,212,0.3)]";

    return (
        <aside className="w-96 flex flex-col bg-[#0f172a] border-r border-[#1e293b] z-30 shadow-[4px_0_24px_-12px_rgba(0,0,0,0.5)] h-full text-slate-300 font-sans">

            {/* Header */}
            <div className="p-6 pb-4 flex flex-col items-center justify-center border-b border-[#1e293b]/50 bg-[#0f172a]">
                <div className="w-14 h-14 mb-2 rounded-2xl bg-gradient-to-tr from-cyan-500/20 to-blue-600/20 border border-cyan-500/30 flex items-center justify-center">
                    {mazeMode ? <IconMaze className="w-7 h-7 text-cyan-400" /> : <IconCar className="w-7 h-7 text-cyan-400" />}
                </div>
                <h1 className="text-lg font-black tracking-[0.15em] text-white uppercase">
                    Maze Solver <span className="text-cyan-400">3D</span>
                </h1>
            </div>

            {/* Tab Navigation */}
            <div className="flex border-b border-[#1e293b]">
                {['controls', 'generator', 'history'].map(tab => (
                    <button
                        key={tab}
                        onClick={() => setActiveTab(tab)}
                        className={`flex-1 py-3 text-[10px] font-bold uppercase tracking-wider transition-colors ${activeTab === tab
                            ? 'text-cyan-400 border-b-2 border-cyan-400 bg-cyan-500/5'
                            : 'text-slate-500 hover:text-slate-300'
                            }`}
                    >
                        {tab === 'controls' && '⚙️ '}
                        {tab === 'generator' && '🏗️ '}
                        {tab === 'history' && '📊 '}
                        {tab}
                    </button>
                ))}
            </div>

            <div className="flex-1 overflow-y-auto p-4 space-y-6 scrollbar-thin scrollbar-thumb-slate-700 scrollbar-track-transparent">

                {activeTab === 'controls' && (
                    <>
                        {/* Mode Toggle */}
                        <div className="space-y-3">
                            <h2 className="text-[10px] text-slate-500 uppercase tracking-[0.2em] font-bold">Mode</h2>
                            <div className="bg-[#1e293b]/50 p-1 rounded-lg flex border border-[#334155]/50">
                                <button
                                    onClick={() => onMazeModeChange?.(true)}
                                    className={`flex-1 py-2 text-[10px] font-bold uppercase tracking-wider rounded-md transition-all ${mazeMode ? "bg-cyan-500/20 text-cyan-300" : "text-slate-500 hover:text-slate-300"
                                        }`}
                                >
                                    🧩 Maze
                                </button>
                                <button
                                    onClick={() => onMazeModeChange?.(false)}
                                    className={`flex-1 py-2 text-[10px] font-bold uppercase tracking-wider rounded-md transition-all ${!mazeMode ? "bg-cyan-500/20 text-cyan-300" : "text-slate-500 hover:text-slate-300"
                                        }`}
                                >
                                    🚗 Free
                                </button>
                            </div>
                        </div>

                        {/* Connection */}
                        <div className="space-y-3">
                            <h2 className="text-[10px] text-slate-500 uppercase tracking-[0.2em] font-bold">Connection</h2>

                            {error && (
                                <div className="p-2 bg-red-900/20 border border-red-500/30 rounded-lg flex gap-2 items-center">
                                    <div className="w-2 h-2 rounded-full bg-red-500 animate-pulse" />
                                    <p className="text-xs text-red-400 font-mono">{error}</p>
                                </div>
                            )}

                            <div className="space-y-2">
                                <input
                                    type="text"
                                    value={ipAddress}
                                    onChange={(e) => setIpAddress(e.target.value)}
                                    placeholder="192.168.1.100"
                                    disabled={isConnected || isConnecting}
                                    className={`w-full bg-[#1e293b]/50 border rounded-lg py-2 px-3 text-sm font-mono text-cyan-300 placeholder-slate-600 outline-none transition-all ${borderGlow}`}
                                />
                                <div className="flex gap-2">
                                    <input
                                        type="text"
                                        value={port}
                                        onChange={(e) => setPort(e.target.value)}
                                        placeholder="81"
                                        disabled={isConnected || isConnecting}
                                        className={`w-20 bg-[#1e293b]/50 border rounded-lg py-2 px-3 text-center text-sm font-mono text-cyan-300 placeholder-slate-600 outline-none transition-all ${borderGlow}`}
                                    />
                                    <button
                                        onClick={isConnected ? onDisconnect : handleConnect}
                                        disabled={!isConnected && isDemoMode}
                                        className={`flex-1 py-2 rounded-lg text-xs font-bold uppercase tracking-wider transition-all border ${isConnected
                                            ? "bg-red-500/10 border-red-500/50 text-red-400 hover:bg-red-500/20"
                                            : "bg-cyan-500/10 border-cyan-500/50 text-cyan-400 hover:bg-cyan-500/20"
                                            }`}
                                    >
                                        {isConnected ? "Disconnect" : isConnecting ? "Connecting..." : "Connect"}
                                    </button>
                                </div>
                            </div>
                        </div>

                        {/* Camera Toggle */}
                        <div className="space-y-3">
                            <h2 className="text-[10px] text-slate-500 uppercase tracking-[0.2em] font-bold">Camera</h2>
                            <div className="bg-[#1e293b]/50 p-1 rounded-lg flex border border-[#334155]/50">
                                <button
                                    onClick={() => onCameraChange?.('first')}
                                    className={`flex-1 py-2 text-[10px] font-bold uppercase tracking-wider rounded-md transition-all ${cameraMode === 'first' ? "bg-cyan-500/20 text-cyan-300" : "text-slate-500 hover:text-slate-300"
                                        }`}
                                >
                                    First Person
                                </button>
                                <button
                                    onClick={() => onCameraChange?.('third')}
                                    className={`flex-1 py-2 text-[10px] font-bold uppercase tracking-wider rounded-md transition-all ${cameraMode === 'third' ? "bg-cyan-500/20 text-cyan-300" : "text-slate-500 hover:text-slate-300"
                                        }`}
                                >
                                    Third Person
                                </button>
                            </div>
                        </div>

                        {/* Demo Button */}
                        <button
                            onClick={() => isDemoMode ? stopDemo() : startMazeDemo()}
                            className={`w-full py-3 rounded-lg text-sm font-bold uppercase tracking-[0.1em] transition-all border relative overflow-hidden ${isDemoMode
                                ? "bg-red-500/10 border-red-500 text-red-400"
                                : "bg-[#1e293b]/50 border-slate-700 text-slate-400 hover:border-cyan-500/50 hover:text-cyan-300"
                                }`}
                        >
                            {isDemoMode && <div className="absolute inset-0 bg-red-500/5 animate-pulse" />}
                            <span className="relative z-10">{isDemoMode ? 'Stop Demo' : '▶ Start Demo'}</span>
                        </button>

                        {/* Telemetry (Maze Mode) */}
                        {mazeMode && mazeData && (
                            <div className="space-y-3">
                                <h2 className="text-[10px] text-slate-500 uppercase tracking-[0.2em] font-bold">Telemetry</h2>
                                <div className="grid grid-cols-2 gap-3 bg-[#1e293b]/30 p-3 rounded-xl border border-[#334155]/30">
                                    <Gauge value={mazeData.heading || 0} min={0} max={360} label="Heading" unit="°" color="#06b6d4" />
                                    <Gauge
                                        value={Math.abs(mazeData.step || 0)}
                                        min={0}
                                        max={100}
                                        label="Steps"
                                        color={mazeData.isGoal ? '#10b981' : '#8b5cf6'}
                                    />
                                </div>
                            </div>
                        )}

                        {/* Telemetry (Free Mode) */}
                        {!mazeMode && (
                            <div className="space-y-3">
                                <h2 className="text-[10px] text-slate-500 uppercase tracking-[0.2em] font-bold">Telemetry</h2>
                                <div className="grid grid-cols-2 gap-3 bg-[#1e293b]/30 p-3 rounded-xl border border-[#334155]/30">
                                    <Gauge value={sensorData?.yaw || 0} min={0} max={360} label="Heading" unit="°" color="#06b6d4" />
                                    <Gauge
                                        value={Math.abs(sensorData?.speed || 0) * 100}
                                        min={0} max={100}
                                        label="Power"
                                        unit="%"
                                        color={sensorData?.direction === 'backward' ? '#f43f5e' : '#10b981'}
                                    />
                                </div>
                            </div>
                        )}
                    </>
                )}

                {activeTab === 'generator' && (
                    <MazePatternGenerator
                        startCorner={startCorner}
                        goalCorner={goalCorner}
                        walls={mazeWalls}
                        onStartCornerChange={onStartCornerChange}
                        onGoalCornerChange={onGoalCornerChange}
                        onWallsChange={onMazeWallsChange}
                        onMazeChange={onMazeChange}
                    />
                )}

                {activeTab === 'history' && (
                    <div className="space-y-3">
                        <h2 className="text-[10px] text-slate-500 uppercase tracking-[0.2em] font-bold">Episode History</h2>
                        {episodeHistory.length === 0 ? (
                            <p className="text-slate-500 text-sm text-center py-8">No episodes yet</p>
                        ) : (
                            <div className="space-y-2">
                                {episodeHistory.slice(-10).reverse().map((ep, i) => (
                                    <div
                                        key={i}
                                        className={`p-3 rounded-lg border ${ep.solved
                                            ? 'bg-emerald-900/20 border-emerald-500/30'
                                            : 'bg-slate-800/50 border-slate-700'
                                            }`}
                                    >
                                        <div className="flex justify-between items-center">
                                            <span className="text-xs font-bold text-slate-300">Episode {ep.episode}</span>
                                            {ep.solved && <span className="text-xs text-emerald-400">✓ Solved</span>}
                                        </div>
                                        <div className="flex justify-between mt-1 text-xs text-slate-500">
                                            <span>{ep.steps} steps</span>
                                            <span className={ep.reward >= 0 ? 'text-emerald-400' : 'text-red-400'}>
                                                {ep.reward > 0 ? '+' : ''}{ep.reward?.toFixed(0)}
                                            </span>
                                        </div>
                                    </div>
                                ))}
                            </div>
                        )}
                    </div>
                )}
            </div>
        </aside>
    );
}

export default Sidebar;
