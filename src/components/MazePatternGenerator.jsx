import { useState, useCallback, useEffect } from 'react';

/**
 * Maze Pattern Generator
 * Interactive tool to design 5x5 mazes for physical construction
 */

const MAZE_SIZE = 5;

// Corner positions
const CORNERS = {
    topLeft: { x: 0, y: 0, arrow: '↓' },
    topRight: { x: 4, y: 0, arrow: '↓' },
    bottomLeft: { x: 0, y: 4, arrow: '↑' },
    bottomRight: { x: 4, y: 4, arrow: '↑' },
};

function initEmptyMaze() {
    const maze = [];
    for (let y = 0; y < MAZE_SIZE; y++) {
        maze[y] = [];
        for (let x = 0; x < MAZE_SIZE; x++) {
            maze[y][x] = { north: false, east: false, south: false, west: false };
        }
    }
    return maze;
}

export function MazePatternGenerator({
    onMazeChange,
    startCorner: externalStartCorner,
    goalCorner: externalGoalCorner,
    onStartCornerChange,
    onGoalCornerChange,
    walls: externalWalls,
    onWallsChange
}) {
    // Use external state if provided, otherwise local state
    const [localWalls, setLocalWalls] = useState(() => initEmptyMaze());
    const [localStartCorner, setLocalStartCorner] = useState('topLeft');
    const [localGoalCorner, setLocalGoalCorner] = useState('bottomRight');

    // Determine which state to use (prefer external/lifted state)
    const walls = externalWalls || localWalls;
    const startCorner = externalStartCorner || localStartCorner;
    const goalCorner = externalGoalCorner || localGoalCorner;

    const setWalls = onWallsChange || setLocalWalls;
    const setStartCorner = onStartCornerChange || setLocalStartCorner;
    const setGoalCorner = onGoalCornerChange || setLocalGoalCorner;

    // Notify parent of start/goal changes
    const handleStartCornerChange = useCallback((value) => {
        setStartCorner(value);
        onMazeChange?.(walls, CORNERS[value], CORNERS[goalCorner]);
    }, [walls, goalCorner, setStartCorner, onMazeChange]);

    const handleGoalCornerChange = useCallback((value) => {
        setGoalCorner(value);
        onMazeChange?.(walls, CORNERS[startCorner], CORNERS[value]);
    }, [walls, startCorner, setGoalCorner, onMazeChange]);

    const toggleWall = useCallback((x, y, direction) => {
        const updated = JSON.parse(JSON.stringify(walls));
        updated[y][x][direction] = !updated[y][x][direction];

        // Also toggle adjacent cell's opposite wall
        const opposites = { north: 'south', south: 'north', east: 'west', west: 'east' };
        const offsets = { north: [0, -1], south: [0, 1], east: [1, 0], west: [-1, 0] };
        const [dx, dy] = offsets[direction];
        const nx = x + dx;
        const ny = y + dy;

        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            updated[ny][nx][opposites[direction]] = updated[y][x][direction];
        }

        setWalls(updated);
        onMazeChange?.(updated, CORNERS[startCorner], CORNERS[goalCorner]);
    }, [walls, startCorner, goalCorner, setWalls, onMazeChange]);

    const generateRandomMaze = useCallback(() => {
        const maze = initEmptyMaze();

        // Simple random maze: add walls randomly
        for (let y = 0; y < MAZE_SIZE; y++) {
            for (let x = 0; x < MAZE_SIZE; x++) {
                // Random walls (30% chance each)
                if (Math.random() < 0.3 && y > 0) {
                    maze[y][x].north = true;
                    maze[y - 1][x].south = true;
                }
                if (Math.random() < 0.3 && x < MAZE_SIZE - 1) {
                    maze[y][x].east = true;
                    maze[y][x + 1].west = true;
                }
            }
        }

        // Ensure start and goal are accessible
        const start = CORNERS[startCorner];
        const goal = CORNERS[goalCorner];
        maze[start.y][start.x] = { north: false, east: false, south: false, west: false };
        maze[goal.y][goal.x] = { north: false, east: false, south: false, west: false };

        setWalls(maze);
        onMazeChange?.(maze, CORNERS[startCorner], CORNERS[goalCorner]);
    }, [startCorner, goalCorner, setWalls, onMazeChange]);

    const clearMaze = useCallback(() => {
        const maze = initEmptyMaze();
        setWalls(maze);
        onMazeChange?.(maze, CORNERS[startCorner], CORNERS[goalCorner]);
    }, [startCorner, goalCorner, setWalls, onMazeChange]);

    const exportMaze = useCallback(() => {
        const data = {
            walls,
            start: CORNERS[startCorner],
            goal: CORNERS[goalCorner],
        };
        const json = JSON.stringify(data, null, 2);
        navigator.clipboard.writeText(json);
        alert('Maze pattern copied to clipboard!');
    }, [walls, startCorner, goalCorner]);

    return (
        <div className="space-y-4">
            <h3 className="text-sm font-bold text-slate-300 uppercase tracking-wider">
                🏗️ Maze Pattern Generator
            </h3>

            {/* Maze Grid Editor */}
            <div className="flex justify-center">
                <div className="relative">
                    {/* Grid */}
                    <div className="grid gap-0" style={{ gridTemplateColumns: `repeat(${MAZE_SIZE}, 44px)` }}>
                        {Array.from({ length: MAZE_SIZE }).map((_, y) =>
                            Array.from({ length: MAZE_SIZE }).map((_, x) => {
                                const isStart = CORNERS[startCorner].x === x && CORNERS[startCorner].y === y;
                                const isGoal = CORNERS[goalCorner].x === x && CORNERS[goalCorner].y === y;
                                const cell = walls[y][x];

                                return (
                                    <div
                                        key={`${x}-${y}`}
                                        className={`relative w-11 h-11 border border-slate-600 ${isStart ? 'bg-emerald-900/50' :
                                            isGoal ? 'bg-amber-900/50' : 'bg-slate-800'
                                            }`}
                                    >
                                        {/* Cell walls - clickable */}
                                        {/* North wall */}
                                        <button
                                            className={`absolute top-0 left-1 right-1 h-1.5 rounded-full transition-colors ${cell.north ? 'bg-cyan-400' : 'bg-slate-700 hover:bg-slate-500'
                                                }`}
                                            onClick={() => toggleWall(x, y, 'north')}
                                        />
                                        {/* South wall */}
                                        <button
                                            className={`absolute bottom-0 left-1 right-1 h-1.5 rounded-full transition-colors ${cell.south ? 'bg-cyan-400' : 'bg-slate-700 hover:bg-slate-500'
                                                }`}
                                            onClick={() => toggleWall(x, y, 'south')}
                                        />
                                        {/* East wall */}
                                        <button
                                            className={`absolute right-0 top-1 bottom-1 w-1.5 rounded-full transition-colors ${cell.east ? 'bg-cyan-400' : 'bg-slate-700 hover:bg-slate-500'
                                                }`}
                                            onClick={() => toggleWall(x, y, 'east')}
                                        />
                                        {/* West wall */}
                                        <button
                                            className={`absolute left-0 top-1 bottom-1 w-1.5 rounded-full transition-colors ${cell.west ? 'bg-cyan-400' : 'bg-slate-700 hover:bg-slate-500'
                                                }`}
                                            onClick={() => toggleWall(x, y, 'west')}
                                        />

                                        {/* Start/Goal labels */}
                                        {isStart && (
                                            <span className="absolute inset-0 flex items-center justify-center text-emerald-400 font-bold text-sm">
                                                S
                                            </span>
                                        )}
                                        {isGoal && (
                                            <span className="absolute inset-0 flex items-center justify-center text-amber-400 font-bold text-lg">
                                                ★
                                            </span>
                                        )}
                                    </div>
                                );
                            })
                        )}
                    </div>
                </div>
            </div>

            {/* Corner Selectors */}
            <div className="grid grid-cols-2 gap-3">
                <div>
                    <label className="text-xs text-slate-500 uppercase">Start Corner</label>
                    <select
                        value={startCorner}
                        onChange={(e) => handleStartCornerChange(e.target.value)}
                        className="w-full mt-1 bg-slate-800 border border-slate-600 rounded px-2 py-1.5 text-sm text-slate-300"
                    >
                        <option value="topLeft">Top Left</option>
                        <option value="topRight">Top Right</option>
                        <option value="bottomLeft">Bottom Left</option>
                        <option value="bottomRight">Bottom Right</option>
                    </select>
                </div>
                <div>
                    <label className="text-xs text-slate-500 uppercase">Goal Corner</label>
                    <select
                        value={goalCorner}
                        onChange={(e) => handleGoalCornerChange(e.target.value)}
                        className="w-full mt-1 bg-slate-800 border border-slate-600 rounded px-2 py-1.5 text-sm text-slate-300"
                    >
                        <option value="topLeft">Top Left</option>
                        <option value="topRight">Top Right</option>
                        <option value="bottomLeft">Bottom Left</option>
                        <option value="bottomRight">Bottom Right</option>
                    </select>
                </div>
            </div>

            {/* Action Buttons */}
            <div className="flex gap-2">
                <button
                    onClick={generateRandomMaze}
                    className="flex-1 py-2 px-3 bg-cyan-600/20 border border-cyan-500/50 text-cyan-400 rounded-lg text-xs font-bold uppercase hover:bg-cyan-600/30 transition-colors"
                >
                    🎲 Random
                </button>
                <button
                    onClick={clearMaze}
                    className="flex-1 py-2 px-3 bg-slate-700/50 border border-slate-600 text-slate-400 rounded-lg text-xs font-bold uppercase hover:bg-slate-700 transition-colors"
                >
                    🗑️ Clear
                </button>
                <button
                    onClick={exportMaze}
                    className="flex-1 py-2 px-3 bg-emerald-600/20 border border-emerald-500/50 text-emerald-400 rounded-lg text-xs font-bold uppercase hover:bg-emerald-600/30 transition-colors"
                >
                    📋 Export
                </button>
            </div>

            {/* Legend */}
            <div className="pt-3 border-t border-slate-700">
                <p className="text-xs text-slate-500 mb-2">Click wall edges to toggle walls.</p>
                <div className="flex gap-4 text-xs">
                    <span className="text-emerald-400">S = Start (double tape)</span>
                    <span className="text-amber-400">★ = Goal (triple tape)</span>
                </div>
            </div>
        </div>
    );
}

export default MazePatternGenerator;
