import { useMemo } from 'react';
import * as THREE from 'three';

/**
 * 3D Maze Component
 * Creates walls based on a grid layout
 */
export function Maze({ layout, cellSize = 2, wallHeight = 1.5 }) {
    const walls = useMemo(() => {
        const wallList = [];
        const rows = layout.length;
        const cols = layout[0].length;

        // Offset to center the maze
        const offsetX = -(cols * cellSize) / 2 + cellSize / 2;
        const offsetZ = -(rows * cellSize) / 2 + cellSize / 2;

        for (let row = 0; row < rows; row++) {
            for (let col = 0; col < cols; col++) {
                if (layout[row][col] === 1) {
                    wallList.push({
                        position: [
                            col * cellSize + offsetX,
                            wallHeight / 2,
                            row * cellSize + offsetZ
                        ],
                        key: `wall-${row}-${col}`,
                    });
                }
            }
        }

        return wallList;
    }, [layout, cellSize, wallHeight]);

    return (
        <group>
            {walls.map(({ position, key }) => (
                <Wall
                    key={key}
                    position={position}
                    size={[cellSize * 0.95, wallHeight, cellSize * 0.95]}
                />
            ))}
        </group>
    );
}

/**
 * Individual wall block with glowing edges
 */
function Wall({ position, size }) {
    return (
        <group position={position}>
            {/* Main wall block */}
            <mesh castShadow receiveShadow>
                <boxGeometry args={size} />
                <meshStandardMaterial
                    color="#1e293b"
                    metalness={0.3}
                    roughness={0.7}
                />
            </mesh>

            {/* Top edge glow */}
            <mesh position={[0, size[1] / 2, 0]}>
                <boxGeometry args={[size[0], 0.05, size[2]]} />
                <meshStandardMaterial
                    color="#06b6d4"
                    emissive="#06b6d4"
                    emissiveIntensity={0.5}
                />
            </mesh>

            {/* Edge lines for visual clarity */}
            <lineSegments>
                <edgesGeometry args={[new THREE.BoxGeometry(...size)]} />
                <lineBasicMaterial color="#0ea5e9" linewidth={1} />
            </lineSegments>
        </group>
    );
}

/**
 * Pre-defined maze layouts
 */
export const MAZE_LAYOUTS = {
    simple: [
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 1, 1, 0, 1, 1, 0, 1],
        [1, 0, 1, 0, 0, 0, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 0, 0, 0, 1, 0, 1],
        [1, 0, 1, 1, 0, 1, 1, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
    ],

    complex: [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
        [1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1],
        [1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
        [1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
        [1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    ],

    open: [
        [1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1],
    ],
};

export default Maze;
