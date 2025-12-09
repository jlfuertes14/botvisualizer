import { useState, useCallback } from 'react';
import { Scene3D } from './components/Scene3D';
import { Sidebar } from './components/Sidebar';
import { HUD } from './components/HUD';
import { useWebSocket } from './hooks/useWebSocket';

// Corner positions (same as MazePatternGenerator)
const CORNERS = {
  topLeft: { x: 0, y: 0 },
  topRight: { x: 4, y: 0 },
  bottomLeft: { x: 0, y: 4 },
  bottomRight: { x: 4, y: 4 },
};

function initEmptyMaze() {
  const maze = [];
  for (let y = 0; y < 5; y++) {
    maze[y] = [];
    for (let x = 0; x < 5; x++) {
      maze[y][x] = { north: false, east: false, south: false, west: false };
    }
  }
  return maze;
}

function App() {
  const {
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
    resetMazeData
  } = useWebSocket();

  const [cameraMode, setCameraMode] = useState('third');
  const [mazeMode, setMazeMode] = useState(true);

  // Lifted state from MazePatternGenerator - persists across tab switches
  const [startCorner, setStartCorner] = useState('topLeft');
  const [goalCorner, setGoalCorner] = useState('bottomRight');
  const [mazeWalls, setMazeWalls] = useState(() => initEmptyMaze());

  // Compute start/goal cells from corner names
  const startCell = CORNERS[startCorner];
  const goalCell = CORNERS[goalCorner];

  // Handle maze changes from the generator
  const handleMazeChange = useCallback((walls, start, goal) => {
    // walls, start, goal are passed from MazePatternGenerator
    // We update our lifted state accordingly
  }, []);

  const connectionStatus = isConnected ? 'connected' :
    isConnecting ? 'connecting' : 'disconnected';

  return (
    <div className="flex h-screen w-full bg-slate-900 overflow-hidden font-sans">
      {/* LEFT SIDEBAR */}
      <Sidebar
        sensorData={sensorData}
        mazeData={mazeData}
        episodeHistory={episodeHistory}
        isConnected={isConnected}
        isConnecting={isConnecting}
        error={error}
        onConnect={connect}
        onDisconnect={disconnect}
        onDemoData={setSensorData}
        onDemoMazeData={setMazeData}
        onResetMaze={resetMazeData}
        cameraMode={cameraMode}
        onCameraChange={setCameraMode}
        mazeMode={mazeMode}
        onMazeModeChange={setMazeMode}
        startCell={startCell}
        goalCell={goalCell}
        // Lifted maze generator state
        startCorner={startCorner}
        goalCorner={goalCorner}
        mazeWalls={mazeWalls}
        onStartCornerChange={setStartCorner}
        onGoalCornerChange={setGoalCorner}
        onMazeWallsChange={setMazeWalls}
        onMazeChange={handleMazeChange}
      />

      {/* MAIN 3D VIEW */}
      <main className="flex-1 relative bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900">
        {/* Status Overlay */}
        <div className={`absolute top-4 right-4 flex items-center gap-2 px-4 py-2 rounded-full z-10 shadow-lg backdrop-blur-sm
          ${connectionStatus === 'connected' ? 'bg-emerald-900/50 border border-emerald-500/50' :
            connectionStatus === 'connecting' ? 'bg-amber-900/50 border border-amber-500/50' :
              'bg-slate-800/50 border border-slate-600'}`}>
          <div className={`w-2 h-2 rounded-full
            ${connectionStatus === 'connected' ? 'bg-emerald-400 animate-pulse' :
              connectionStatus === 'connecting' ? 'bg-amber-400 animate-pulse' : 'bg-slate-500'}`} />
          <span className={`text-xs font-medium
            ${connectionStatus === 'connected' ? 'text-emerald-300' :
              connectionStatus === 'connecting' ? 'text-amber-300' : 'text-slate-400'}`}>
            {connectionStatus === 'connected' ? 'Connected' :
              connectionStatus === 'connecting' ? 'Connecting...' : 'Offline'}
          </span>
        </div>

        {/* Mode Indicator */}
        <div className="absolute top-4 left-4 flex gap-2 z-10">
          <div className="px-4 py-2 rounded-full bg-slate-800/50 border border-slate-600 backdrop-blur-sm">
            <span className="text-xs text-slate-300 font-medium">
              {mazeMode ? '🧩 Maze Mode' : '🚗 Free Roam'}
            </span>
          </div>
          <div className="px-4 py-2 rounded-full bg-slate-800/50 border border-slate-600 backdrop-blur-sm">
            <span className="text-xs text-slate-300 font-medium">
              {cameraMode === 'first' ? '👁️ First Person' : '🎥 Third Person'}
            </span>
          </div>
        </div>

        {/* Episode Stats (Maze Mode) */}
        {mazeMode && mazeData && (
          <div className="absolute top-16 right-4 bg-slate-800/80 border border-slate-600 rounded-xl p-4 z-10 backdrop-blur-sm">
            <div className="text-xs text-slate-400 uppercase tracking-wider mb-2">Episode Stats</div>
            <div className="grid grid-cols-2 gap-3 text-sm">
              <div>
                <div className="text-slate-500 text-xs">Episode</div>
                <div className="text-cyan-400 font-bold">{mazeData.episode || 0}</div>
              </div>
              <div>
                <div className="text-slate-500 text-xs">Steps</div>
                <div className="text-white font-bold">{mazeData.step || 0}</div>
              </div>
              <div>
                <div className="text-slate-500 text-xs">Reward</div>
                <div className={`font-bold ${(mazeData.reward || 0) >= 0 ? 'text-emerald-400' : 'text-red-400'}`}>
                  {mazeData.reward?.toFixed(1) || 0}
                </div>
              </div>
              <div>
                <div className="text-slate-500 text-xs">Position</div>
                <div className="text-white font-mono text-xs">
                  ({mazeData.position?.x || 0}, {mazeData.position?.y || 0})
                </div>
              </div>
            </div>
            {mazeData.isGoal && (
              <div className="mt-3 py-2 bg-amber-500/20 border border-amber-500/50 rounded text-center text-amber-400 text-xs font-bold animate-pulse">
                🎉 GOAL REACHED!
              </div>
            )}
          </div>
        )}

        {/* HUD */}
        {!mazeMode && <HUD sensorData={sensorData} />}

        {/* 3D Canvas */}
        <div className="w-full h-full">
          <Scene3D
            sensorData={sensorData}
            cameraMode={cameraMode}
            mazeMode={mazeMode}
            mazeData={mazeData}
            discoveredWalls={discoveredWalls}
            pathHistory={pathHistory}
            startCell={startCell}
            goalCell={goalCell}
          />
        </div>

        {/* Subtle vignette */}
        <div className="absolute inset-0 pointer-events-none bg-[radial-gradient(ellipse_at_center,transparent_60%,rgba(15,23,42,0.6)_100%)]" />
      </main>
    </div>
  );
}

export default App;
