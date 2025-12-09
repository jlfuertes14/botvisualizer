import { useRef, useMemo } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { Grid, Environment } from '@react-three/drei';
import * as THREE from 'three';
import { MazeGrid } from './MazeGrid';

const CELL_SIZE = 2;

export function Scene3D({
    sensorData,
    cameraMode = 'third',
    mazeMode = false,
    mazeData = null,
    discoveredWalls = {},
    pathHistory = [],
    startCell = { x: 0, y: 0 },
    goalCell = { x: 4, y: 4 }
}) {
    return (
        <div className="w-full h-full">
            <Canvas shadows camera={{ position: [8, 10, 8], fov: 50 }}>
                <ambientLight intensity={0.7} />
                <directionalLight position={[10, 20, 10]} intensity={1.5} castShadow color="#ffffff" />
                <directionalLight position={[-10, 10, -10]} intensity={0.3} color="#f0f9ff" />
                <hemisphereLight args={['#ffffff', '#e5e7eb', 0.5]} />
                <Environment preset="apartment" />
                <fog attach="fog" args={['#0f172a', 30, 100]} />

                {mazeMode ? (
                    <>
                        {/* Maze Grid with walls and path */}
                        <MazeGrid
                            discoveredWalls={discoveredWalls}
                            pathHistory={pathHistory}
                            robotPosition={mazeData?.position || { x: 0, y: 0 }}
                            startCell={startCell}
                            goalCell={goalCell}
                            mazeSize={5}
                        />

                        {/* Robot with cell-based positioning */}
                        <MazeRobotWithCamera
                            mazeData={mazeData}
                            sensorData={sensorData}
                            cameraMode={cameraMode}
                        />
                    </>
                ) : (
                    <>
                        {/* Original infinite grid */}
                        <Grid
                            args={[100, 100]}
                            cellSize={2}
                            cellThickness={0.4}
                            cellColor="#64748b"
                            sectionSize={10}
                            sectionThickness={1}
                            sectionColor="#94a3b8"
                            fadeDistance={60}
                            followCamera={false}
                            position={[0, 0, 0]}
                            infiniteGrid
                        />

                        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]} receiveShadow>
                            <planeGeometry args={[100, 100]} />
                            <meshStandardMaterial color="#1e293b" />
                        </mesh>

                        <RobotWithCamera sensorData={sensorData} cameraMode={cameraMode} />
                    </>
                )}
            </Canvas>
        </div>
    );
}

// Robot that moves cell-by-cell in maze mode
function MazeRobotWithCamera({ mazeData, sensorData, cameraMode }) {
    const { camera } = useThree();
    const robotRef = useRef();
    const posRef = useRef(new THREE.Vector3(0, 0, 0));
    const targetPosRef = useRef(new THREE.Vector3(0, 0, 0));
    const yawRef = useRef(0);

    // Calculate target position from maze data
    const mazeSize = 5;
    const offset = -(mazeSize * CELL_SIZE) / 2 + CELL_SIZE / 2;

    useFrame((_, delta) => {
        const position = mazeData?.position || { x: 0, y: 0 };
        const heading = mazeData?.heading || sensorData?.yaw || 0;

        // Target position based on cell coordinates
        targetPosRef.current.x = position.x * CELL_SIZE + offset;
        targetPosRef.current.z = position.y * CELL_SIZE + offset;

        // Smooth interpolation to target
        posRef.current.x = THREE.MathUtils.lerp(posRef.current.x, targetPosRef.current.x, 0.1);
        posRef.current.z = THREE.MathUtils.lerp(posRef.current.z, targetPosRef.current.z, 0.1);

        // Smooth yaw rotation
        yawRef.current = THREE.MathUtils.lerp(yawRef.current, heading, 0.1);
        const yawRad = THREE.MathUtils.degToRad(-yawRef.current);

        if (robotRef.current) {
            robotRef.current.position.x = posRef.current.x;
            robotRef.current.position.z = posRef.current.z;
            robotRef.current.rotation.y = yawRad;

            // Subtle bobbing when moving
            const isMoving = Math.abs(posRef.current.x - targetPosRef.current.x) > 0.1 ||
                Math.abs(posRef.current.z - targetPosRef.current.z) > 0.1;
            robotRef.current.position.y = isMoving ? Math.sin(Date.now() / 100) * 0.015 : 0;
        }

        // Camera follow
        if (cameraMode === 'first') {
            camera.position.x = THREE.MathUtils.lerp(camera.position.x, posRef.current.x, 0.15);
            camera.position.y = THREE.MathUtils.lerp(camera.position.y, 1.0, 0.15);
            camera.position.z = THREE.MathUtils.lerp(camera.position.z, posRef.current.z, 0.15);
            camera.lookAt(posRef.current.x + Math.sin(yawRad) * 20, 0.5, posRef.current.z + Math.cos(yawRad) * 20);
        } else {
            // Top-down isometric view for maze
            const camX = posRef.current.x - 4;
            const camZ = posRef.current.z + 4;
            camera.position.x = THREE.MathUtils.lerp(camera.position.x, camX, 0.05);
            camera.position.y = THREE.MathUtils.lerp(camera.position.y, 10, 0.05);
            camera.position.z = THREE.MathUtils.lerp(camera.position.z, camZ, 0.05);
            camera.lookAt(posRef.current.x, 0, posRef.current.z);
        }
    });

    return (
        <group ref={robotRef}>
            <SmartCarRobot direction={sensorData?.direction || 'stop'} yaw={sensorData?.yaw || 0} />

            {/* Goal celebration effect */}
            {mazeData?.isGoal && (
                <mesh position={[0, 2, 0]}>
                    <sphereGeometry args={[0.3, 16, 16]} />
                    <meshBasicMaterial color="#fbbf24" />
                    <pointLight color="#fbbf24" intensity={2} distance={5} />
                </mesh>
            )}
        </group>
    );
}

// Original continuous movement robot
function RobotWithCamera({ sensorData, cameraMode }) {
    const { camera } = useThree();
    const robotRef = useRef();
    const posRef = useRef(new THREE.Vector3(0, 0, 0));
    const yawRef = useRef(0);

    useFrame((_, delta) => {
        const { yaw, direction, speed } = sensorData;
        yawRef.current = THREE.MathUtils.lerp(yawRef.current, yaw, 0.1);
        const yawRad = THREE.MathUtils.degToRad(-yawRef.current);

        if (direction !== 'stop') {
            const moveSpeed = 5 * delta * (Math.abs(speed) || 0.6);
            const dx = Math.sin(yawRad) * moveSpeed;
            const dz = Math.cos(yawRad) * moveSpeed;
            if (direction === 'forward') { posRef.current.x += dx; posRef.current.z += dz; }
            else { posRef.current.x -= dx; posRef.current.z -= dz; }
        }

        if (robotRef.current) {
            robotRef.current.position.x = posRef.current.x;
            robotRef.current.position.z = posRef.current.z;
            robotRef.current.rotation.y = yawRad;
            robotRef.current.position.y = direction !== 'stop' ? Math.sin(Date.now() / 100) * 0.015 : 0;
        }

        if (cameraMode === 'first') {
            camera.position.x = THREE.MathUtils.lerp(camera.position.x, posRef.current.x, 0.15);
            camera.position.y = THREE.MathUtils.lerp(camera.position.y, 1.0, 0.15);
            camera.position.z = THREE.MathUtils.lerp(camera.position.z, posRef.current.z, 0.15);
            camera.lookAt(posRef.current.x + Math.sin(yawRad) * 20, 0.5, posRef.current.z + Math.cos(yawRad) * 20);
        } else {
            const camX = posRef.current.x - Math.sin(yawRad) * 5;
            const camZ = posRef.current.z - Math.cos(yawRad) * 5;
            camera.position.x = THREE.MathUtils.lerp(camera.position.x, camX, 0.08);
            camera.position.y = THREE.MathUtils.lerp(camera.position.y, 3.5, 0.08);
            camera.position.z = THREE.MathUtils.lerp(camera.position.z, camZ, 0.08);
            camera.lookAt(posRef.current.x, 0, posRef.current.z);
        }
    });

    return (
        <group ref={robotRef}>
            <SmartCarRobot direction={sensorData.direction} yaw={sensorData.yaw} />
        </group>
    );
}

function SmartCarRobot({ direction, yaw }) {
    const leftWheelRef = useRef();
    const rightWheelRef = useRef();
    const prevYawRef = useRef(yaw);

    useFrame((_, delta) => {
        const yawDelta = yaw - prevYawRef.current;
        prevYawRef.current = THREE.MathUtils.lerp(prevYawRef.current, yaw, 0.1);
        const baseSpeed = 8;
        let leftSpeed = 0, rightSpeed = 0;
        const isTurningRight = yawDelta > 0.05;
        const isTurningLeft = yawDelta < -0.05;

        // Visual wheel rotation logic
        if (isTurningRight) { leftSpeed = baseSpeed; rightSpeed = -baseSpeed; }
        else if (isTurningLeft) { leftSpeed = -baseSpeed; rightSpeed = baseSpeed; }
        else if (direction === 'forward') { leftSpeed = baseSpeed; rightSpeed = baseSpeed; }
        else if (direction === 'backward') { leftSpeed = -baseSpeed; rightSpeed = -baseSpeed; }

        if (leftWheelRef.current) leftWheelRef.current.rotation.x += leftSpeed * delta;
        if (rightWheelRef.current) rightWheelRef.current.rotation.x += rightSpeed * delta;
    });

    return (
        <group position={[0, 0.35, 0]}>
            {/* --- CHASSIS --- */}
            {/* Bottom Plate - Clear Acrylic */}
            <mesh position={[0, 0, 0]} castShadow receiveShadow>
                <cylinderGeometry args={[0.7, 0.7, 0.05, 32]} />
                <meshPhysicalMaterial
                    color="#ffffff"
                    metalness={0.1}
                    roughness={0.1}
                    transmission={0.95} // Glass-like
                    thickness={0.5}
                    transparent
                    opacity={0.4}
                />
            </mesh>

            {/* Top Plate - Clear Acrylic */}
            <mesh position={[0, 0.5, 0]} castShadow receiveShadow>
                <cylinderGeometry args={[0.7, 0.7, 0.05, 32]} />
                <meshPhysicalMaterial
                    color="#ffffff"
                    metalness={0.1}
                    roughness={0.1}
                    transmission={0.95}
                    thickness={0.5}
                    transparent
                    opacity={0.4}
                />
            </mesh>

            {/* Brass Standoffs (4x) */}
            {[[-0.3, 0.3], [0.3, 0.3], [-0.3, -0.3], [0.3, -0.3]].map((p, i) => (
                <mesh key={i} position={[p[0], 0.25, p[1]]} castShadow>
                    <cylinderGeometry args={[0.02, 0.02, 0.5, 8]} />
                    <meshStandardMaterial color="#fbbf24" metalness={0.9} roughness={0.2} />
                </mesh>
            ))}

            {/* --- DRIVETRAIN --- */}
            {/* Yellow DC Motors (under bottom plate) */}
            <group position={[0, -0.15, 0]}>
                {/* Left Motor */}
                <mesh position={[-0.2, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
                    <boxGeometry args={[0.25, 0.45, 0.2]} />
                    <meshStandardMaterial color="#facc15" /> {/* Yellow */}
                </mesh>
                <mesh position={[-0.45, 0, 0.05]} rotation={[0, 0, Math.PI / 2]}>
                    <cylinderGeometry args={[0.03, 0.03, 0.1, 8]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.8} />
                </mesh>

                {/* Right Motor */}
                <mesh position={[0.2, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
                    <boxGeometry args={[0.25, 0.45, 0.2]} />
                    <meshStandardMaterial color="#facc15" />
                </mesh>
                <mesh position={[0.45, 0, 0.05]} rotation={[0, 0, Math.PI / 2]}>
                    <cylinderGeometry args={[0.03, 0.03, 0.1, 8]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.8} />
                </mesh>
            </group>

            {/* Wheels - 5 Spoke Design */}
            {/* Left Wheel */}
            <group position={[-0.85, -0.15, 0]} ref={leftWheelRef}>
                <group rotation={[0, 0, Math.PI / 2]}>
                    <mesh castShadow>
                        <cylinderGeometry args={[0.4, 0.4, 0.25, 32]} />
                        <meshStandardMaterial color="#1a1a1a" roughness={0.9} /> {/* Tire */}
                    </mesh>
                    <mesh>
                        <cylinderGeometry args={[0.28, 0.28, 0.26, 32]} />
                        <meshStandardMaterial color="#facc15" /> {/* Rim */}
                    </mesh>
                    {/* Spokes */}
                    {[0, 72, 144, 216, 288].map((rot) => (
                        <mesh key={rot} rotation={[0, THREE.MathUtils.degToRad(rot), 0]} position={[0.15, 0.13, 0]}>
                            <boxGeometry args={[0.2, 0.05, 0.04]} />
                            <meshStandardMaterial color="#facc15" />
                        </mesh>
                    ))}
                    <mesh position={[0, 0.14, 0]}>
                        <cylinderGeometry args={[0.05, 0.05, 0.05]} />
                        <meshStandardMaterial color="#fff" />
                    </mesh>
                </group>
            </group>

            {/* Right Wheel */}
            <group position={[0.85, -0.15, 0]} ref={rightWheelRef}>
                <group rotation={[0, 0, Math.PI / 2]}>
                    <mesh castShadow>
                        <cylinderGeometry args={[0.4, 0.4, 0.25, 32]} />
                        <meshStandardMaterial color="#1a1a1a" roughness={0.9} />
                    </mesh>
                    <mesh>
                        <cylinderGeometry args={[0.28, 0.28, 0.26, 32]} />
                        <meshStandardMaterial color="#facc15" />
                    </mesh>
                    {/* Spokes */}
                    {[0, 72, 144, 216, 288].map((rot) => (
                        <mesh key={rot} rotation={[0, THREE.MathUtils.degToRad(rot), 0]} position={[0.15, 0.13, 0]}>
                            <boxGeometry args={[0.2, 0.05, 0.04]} />
                            <meshStandardMaterial color="#facc15" />
                        </mesh>
                    ))}
                    <mesh position={[0, 0.14, 0]}>
                        <cylinderGeometry args={[0.05, 0.05, 0.05]} />
                        <meshStandardMaterial color="#fff" />
                    </mesh>
                </group>
            </group>

            {/* Caster Wheels (Front & Back) */}
            {[0.5, -0.5].map((zPos, i) => (
                <group key={i} position={[0, -0.3, zPos]}>
                    <mesh position={[0, 0.1, 0]}>
                        <boxGeometry args={[0.1, 0.2, 0.1]} />
                        <meshStandardMaterial color="#cbd5e1" metalness={0.5} />
                    </mesh>
                    <mesh position={[0, -0.1, 0]} rotation={[Math.PI / 2, 0, 0]}>
                        <cylinderGeometry args={[0.12, 0.12, 0.08, 16]} />
                        <meshStandardMaterial color="#ffffff" roughness={0.5} />
                    </mesh>
                </group>
            ))}

            {/* --- ELECTRONICS --- */}

            {/* Battery Box (Bottom, Center) */}
            <mesh position={[0, 0.1, 0]} castShadow>
                <boxGeometry args={[0.4, 0.25, 0.5]} />
                <meshStandardMaterial color="#0f172a" roughness={0.4} />
            </mesh>

            {/* Power Bank (Top Plate, Rear, Vertical) */}
            <group position={[0, 0.9, -0.4]}>
                <mesh castShadow>
                    <boxGeometry args={[0.5, 0.7, 0.2]} />
                    <meshStandardMaterial color="#1e1e1e" roughness={0.2} metalness={0.3} />
                </mesh>
                {/* Ports detail */}
                <mesh position={[0.15, 0.2, 0.101]}>
                    <planeGeometry args={[0.1, 0.05]} />
                    <meshStandardMaterial color="#000" />
                </mesh>
            </group>

            {/* L298N Motor Driver (Top Plate, Rear-ish/Center) */}
            <group position={[0, 0.6, 0.35]}>
                <mesh castShadow>
                    <boxGeometry args={[0.25, 0.15, 0.25]} />
                    <meshStandardMaterial color="#dc2626" /> {/* Red PCB */}
                </mesh>
                <mesh position={[0, 0.1, 0]}>
                    <boxGeometry args={[0.1, 0.2, 0.1]} />
                    <meshStandardMaterial color="#111" /> {/* Heatsink */}
                </mesh>
            </group>

            {/* ESP32 Module (Top Plate, Center) */}
            <group position={[0, 0.55, 0.05]}>
                <mesh>
                    <boxGeometry args={[0.3, 0.02, 0.5]} />
                    <meshStandardMaterial color="#1e1e1e" /> {/* Black PCB */}
                </mesh>
                <mesh position={[0, 0.02, 0]}>
                    <boxGeometry args={[0.15, 0.02, 0.15]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.9} /> {/* Chip */}
                </mesh>
            </group>

            {/* Ultrasonic Sensors (x3) */}
            {/* Front */}
            <group position={[0, 0.55, 0.8]} rotation={[0, 0, 0]}>
                <mesh castShadow>
                    <boxGeometry args={[0.25, 0.1, 0.05]} />
                    <meshStandardMaterial color="#0ea5e9" /> {/* Blue PCB */}
                </mesh>
                <mesh position={[-0.08, 0, 0.05]} rotation={[Math.PI / 2, 0, 0]}>
                    <cylinderGeometry args={[0.06, 0.06, 0.15]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.5} />
                </mesh>
                <mesh position={[0.08, 0, 0.05]} rotation={[Math.PI / 2, 0, 0]}>
                    <cylinderGeometry args={[0.06, 0.06, 0.15]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.5} />
                </mesh>
            </group>

            {/* Left */}
            <group position={[-0.8, 0.55, 0.2]} rotation={[0, Math.PI / 2, 0]}>
                <mesh castShadow>
                    <boxGeometry args={[0.25, 0.1, 0.05]} />
                    <meshStandardMaterial color="#0ea5e9" />
                </mesh>
                <mesh position={[-0.08, 0, 0.05]} rotation={[Math.PI / 2, 0, 0]}>
                    <cylinderGeometry args={[0.06, 0.06, 0.15]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.5} />
                </mesh>
                <mesh position={[0.08, 0, 0.05]} rotation={[Math.PI / 2, 0, 0]}>
                    <cylinderGeometry args={[0.06, 0.06, 0.15]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.5} />
                </mesh>
            </group>

            {/* Right */}
            <group position={[0.8, 0.55, 0.2]} rotation={[0, -Math.PI / 2, 0]}>
                <mesh castShadow>
                    <boxGeometry args={[0.25, 0.1, 0.05]} />
                    <meshStandardMaterial color="#0ea5e9" />
                </mesh>
                <mesh position={[-0.08, 0, 0.05]} rotation={[Math.PI / 2, 0, 0]}>
                    <cylinderGeometry args={[0.06, 0.06, 0.15]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.5} />
                </mesh>
                <mesh position={[0.08, 0, 0.05]} rotation={[Math.PI / 2, 0, 0]}>
                    <cylinderGeometry args={[0.06, 0.06, 0.15]} />
                    <meshStandardMaterial color="#cbd5e1" metalness={0.5} />
                </mesh>
            </group>

            {/* Dynamic Status LEDs */}
            <mesh position={[0, 0.8, 0.8]}>
                <sphereGeometry args={[0.05]} />
                <meshStandardMaterial
                    color={direction === 'forward' ? '#fef08a' : '#333'}
                    emissive={direction === 'forward' ? '#fef08a' : '#000'}
                    emissiveIntensity={2}
                />
            </mesh>
        </group>
    );
}

export default Scene3D;
