import { useMemo } from 'react';
import * as THREE from 'three';

/**
 * 5x5 Maze Grid Component for Wall Follower Visualizer
 * Shows grid, discovered walls, start/goal cells, and robot path
 */

const CELL_SIZE = 2;
const WALL_HEIGHT = 1.2;
const WALL_THICKNESS = 0.1;

export function MazeGrid({
    discoveredWalls = {},
    pathHistory = [],
    robotPosition = { x: 0, y: 0 },
    startCell = { x: 0, y: 0 },
    goalCell = null,
    mazeSize = 5
}) {
    // Calculate grid offset to center the maze
    const offset = -(mazeSize * CELL_SIZE) / 2 + CELL_SIZE / 2;

    return (
        <group>
            {/* Grid Floor */}
            <GridFloor mazeSize={mazeSize} cellSize={CELL_SIZE} />

            {/* Cell Markers */}
            <CellMarkers
                startCell={startCell}
                goalCell={goalCell}
                mazeSize={mazeSize}
                cellSize={CELL_SIZE}
                offset={offset}
            />

            {/* Discovered Walls */}
            <DiscoveredWalls
                discoveredWalls={discoveredWalls}
                cellSize={CELL_SIZE}
                wallHeight={WALL_HEIGHT}
                offset={offset}
            />

            {/* Path Trail */}
            <PathTrail
                pathHistory={pathHistory}
                cellSize={CELL_SIZE}
                offset={offset}
            />

            {/* Outer Walls */}
            <OuterWalls mazeSize={mazeSize} cellSize={CELL_SIZE} />
        </group>
    );
}

// Grid floor with cell lines
function GridFloor({ mazeSize, cellSize }) {
    const totalSize = mazeSize * cellSize;
    const halfSize = totalSize / 2;

    return (
        <group>
            {/* Base floor */}
            <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]} receiveShadow>
                <planeGeometry args={[totalSize, totalSize]} />
                <meshStandardMaterial color="#1a1a2e" />
            </mesh>

            {/* Grid lines */}
            {Array.from({ length: mazeSize + 1 }).map((_, i) => {
                const pos = -halfSize + i * cellSize;
                return (
                    <group key={`grid-${i}`}>
                        {/* Horizontal line */}
                        <mesh position={[0, 0.02, pos]} rotation={[-Math.PI / 2, 0, 0]}>
                            <planeGeometry args={[totalSize, 0.05]} />
                            <meshBasicMaterial color="#3a3a5c" />
                        </mesh>
                        {/* Vertical line */}
                        <mesh position={[pos, 0.02, 0]} rotation={[-Math.PI / 2, 0, Math.PI / 2]}>
                            <planeGeometry args={[totalSize, 0.05]} />
                            <meshBasicMaterial color="#3a3a5c" />
                        </mesh>
                    </group>
                );
            })}
        </group>
    );
}

// Start and goal cell indicators
function CellMarkers({ startCell, goalCell, mazeSize, cellSize, offset }) {
    const halfSize = (mazeSize * cellSize) / 2;

    return (
        <group>
            {/* Start cell - green glow */}
            <mesh
                position={[
                    startCell.x * cellSize - halfSize + cellSize / 2,
                    0.03,
                    startCell.y * cellSize - halfSize + cellSize / 2
                ]}
                rotation={[-Math.PI / 2, 0, 0]}
            >
                <planeGeometry args={[cellSize * 0.9, cellSize * 0.9]} />
                <meshBasicMaterial color="#10b981" transparent opacity={0.3} />
            </mesh>

            {/* Goal cell - gold glow (if set) */}
            {goalCell && (
                <mesh
                    position={[
                        goalCell.x * cellSize - halfSize + cellSize / 2,
                        0.03,
                        goalCell.y * cellSize - halfSize + cellSize / 2
                    ]}
                    rotation={[-Math.PI / 2, 0, 0]}
                >
                    <planeGeometry args={[cellSize * 0.9, cellSize * 0.9]} />
                    <meshBasicMaterial color="#f59e0b" transparent opacity={0.4} />
                </mesh>
            )}
        </group>
    );
}

// Walls discovered by the robot
function DiscoveredWalls({ discoveredWalls, cellSize, wallHeight, offset }) {
    const walls = useMemo(() => {
        const wallList = [];
        const halfCell = cellSize / 2;

        Object.entries(discoveredWalls).forEach(([key, hasWall]) => {
            if (!hasWall) return;

            const [x, y, direction] = key.split(',');
            const cellX = parseInt(x) * cellSize + offset;
            const cellZ = parseInt(y) * cellSize + offset;

            let position, rotation;

            switch (direction) {
                case 'north':
                    position = [cellX, wallHeight / 2, cellZ - halfCell];
                    rotation = [0, 0, 0];
                    break;
                case 'south':
                    position = [cellX, wallHeight / 2, cellZ + halfCell];
                    rotation = [0, 0, 0];
                    break;
                case 'east':
                    position = [cellX + halfCell, wallHeight / 2, cellZ];
                    rotation = [0, Math.PI / 2, 0];
                    break;
                case 'west':
                    position = [cellX - halfCell, wallHeight / 2, cellZ];
                    rotation = [0, Math.PI / 2, 0];
                    break;
                default:
                    return;
            }

            wallList.push({ key, position, rotation });
        });

        return wallList;
    }, [discoveredWalls, cellSize, wallHeight, offset]);

    return (
        <group>
            {walls.map(({ key, position, rotation }) => (
                <mesh key={key} position={position} rotation={rotation} castShadow>
                    <boxGeometry args={[cellSize, wallHeight, 0.15]} />
                    <meshStandardMaterial
                        color="#06b6d4"
                        emissive="#06b6d4"
                        emissiveIntensity={0.2}
                        metalness={0.3}
                        roughness={0.7}
                    />
                </mesh>
            ))}
        </group>
    );
}

// Path trail showing robot's journey
function PathTrail({ pathHistory, cellSize, offset }) {
    // Filter out any invalid path entries and require at least 2 valid points
    const validPoints = useMemo(() => {
        if (!pathHistory || !Array.isArray(pathHistory)) return [];

        return pathHistory
            .filter(pos => pos && typeof pos.x === 'number' && typeof pos.y === 'number')
            .map(pos => {
                const x = pos.x * cellSize + offset;
                const z = pos.y * cellSize + offset;
                const y = 0.1;
                return new THREE.Vector3(x, y, z);
            });
    }, [pathHistory, cellSize, offset]);

    // Need at least 2 points to create a curve
    if (validPoints.length < 2) return null;

    const curve = new THREE.CatmullRomCurve3(validPoints);

    return (
        <mesh>
            <tubeGeometry args={[curve, 64, 0.05, 8, false]} />
            <meshStandardMaterial
                color="#8b5cf6"
                emissive="#8b5cf6"
                emissiveIntensity={0.5}
                transparent
                opacity={0.7}
            />
        </mesh>
    );
}

// Outer maze walls
function OuterWalls({ mazeSize, cellSize }) {
    const totalSize = mazeSize * cellSize;
    const halfSize = totalSize / 2;
    const wallHeight = 0.3;

    return (
        <group>
            {/* North wall */}
            <mesh position={[0, wallHeight / 2, -halfSize]} castShadow>
                <boxGeometry args={[totalSize, wallHeight, 0.2]} />
                <meshStandardMaterial color="#475569" metalness={0.5} roughness={0.5} />
            </mesh>
            {/* South wall */}
            <mesh position={[0, wallHeight / 2, halfSize]} castShadow>
                <boxGeometry args={[totalSize, wallHeight, 0.2]} />
                <meshStandardMaterial color="#475569" metalness={0.5} roughness={0.5} />
            </mesh>
            {/* East wall */}
            <mesh position={[halfSize, wallHeight / 2, 0]} castShadow>
                <boxGeometry args={[0.2, wallHeight, totalSize]} />
                <meshStandardMaterial color="#475569" metalness={0.5} roughness={0.5} />
            </mesh>
            {/* West wall */}
            <mesh position={[-halfSize, wallHeight / 2, 0]} castShadow>
                <boxGeometry args={[0.2, wallHeight, totalSize]} />
                <meshStandardMaterial color="#475569" metalness={0.5} roughness={0.5} />
            </mesh>
        </group>
    );
}

export default MazeGrid;
