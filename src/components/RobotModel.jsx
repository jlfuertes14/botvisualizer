import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

/**
 * 3D Wheeled Robot Model with Movement
 * - Rotates based on yaw value
 * - Position is passed from parent (for maze movement)
 * - Shows direction with visual indicators
 */
export function RobotModel({ yaw = 0, direction = 'stop', position = [0, 0, 0] }) {
    const groupRef = useRef();
    const targetRotation = useRef(0);
    const wheelsRef = useRef([]);

    // Smoothly interpolate rotation and animate wheels
    useFrame((state, delta) => {
        if (groupRef.current) {
            // Convert yaw from degrees to radians (negative for correct rotation direction)
            targetRotation.current = THREE.MathUtils.degToRad(-yaw);

            // Smooth interpolation for rotation
            groupRef.current.rotation.y = THREE.MathUtils.lerp(
                groupRef.current.rotation.y,
                targetRotation.current,
                0.15
            );
        }

        // Animate wheels when moving
        if (direction !== 'stop') {
            wheelsRef.current.forEach(wheel => {
                if (wheel) {
                    const speed = direction === 'forward' ? 8 : -8;
                    wheel.rotation.x += speed * delta;
                }
            });
        }
    });

    // Colors based on direction
    const bodyColor = direction === 'forward' ? '#22c55e' :
        direction === 'backward' ? '#ef4444' : '#06b6d4';

    return (
        <group ref={groupRef} position={position}>
            {/* Robot Body - Main chassis */}
            <mesh position={[0, 0.3, 0]} castShadow>
                <boxGeometry args={[1.0, 0.25, 1.4]} />
                <meshStandardMaterial
                    color={bodyColor}
                    metalness={0.6}
                    roughness={0.3}
                />
            </mesh>

            {/* Top Housing / Sensor Platform */}
            <mesh position={[0, 0.5, -0.1]} castShadow>
                <boxGeometry args={[0.7, 0.15, 0.8]} />
                <meshStandardMaterial
                    color="#1e293b"
                    metalness={0.8}
                    roughness={0.2}
                />
            </mesh>

            {/* MPU6050 Sensor (small box on top) */}
            <mesh position={[0, 0.65, 0]} castShadow>
                <boxGeometry args={[0.2, 0.08, 0.3]} />
                <meshStandardMaterial
                    color="#8b5cf6"
                    metalness={0.5}
                    roughness={0.4}
                    emissive="#8b5cf6"
                    emissiveIntensity={0.4}
                />
            </mesh>

            {/* Front Headlights */}
            <mesh position={[-0.25, 0.35, 0.68]}>
                <sphereGeometry args={[0.06, 16, 16]} />
                <meshStandardMaterial
                    color={direction === 'forward' ? '#fef08a' : '#64748b'}
                    emissive={direction === 'forward' ? '#fef08a' : '#000000'}
                    emissiveIntensity={direction === 'forward' ? 1.5 : 0}
                />
            </mesh>
            <mesh position={[0.25, 0.35, 0.68]}>
                <sphereGeometry args={[0.06, 16, 16]} />
                <meshStandardMaterial
                    color={direction === 'forward' ? '#fef08a' : '#64748b'}
                    emissive={direction === 'forward' ? '#fef08a' : '#000000'}
                    emissiveIntensity={direction === 'forward' ? 1.5 : 0}
                />
            </mesh>

            {/* Rear Taillights */}
            <mesh position={[-0.3, 0.35, -0.68]}>
                <boxGeometry args={[0.12, 0.06, 0.02]} />
                <meshStandardMaterial
                    color={direction === 'backward' ? '#ef4444' : '#7f1d1d'}
                    emissive={direction === 'backward' ? '#ef4444' : '#000000'}
                    emissiveIntensity={direction === 'backward' ? 1.2 : 0}
                />
            </mesh>
            <mesh position={[0.3, 0.35, -0.68]}>
                <boxGeometry args={[0.12, 0.06, 0.02]} />
                <meshStandardMaterial
                    color={direction === 'backward' ? '#ef4444' : '#7f1d1d'}
                    emissive={direction === 'backward' ? '#ef4444' : '#000000'}
                    emissiveIntensity={direction === 'backward' ? 1.2 : 0}
                />
            </mesh>

            {/* Wheels with refs for animation */}
            <Wheel ref={el => wheelsRef.current[0] = el} position={[-0.55, 0.15, 0.45]} />
            <Wheel ref={el => wheelsRef.current[1] = el} position={[0.55, 0.15, 0.45]} />
            <Wheel ref={el => wheelsRef.current[2] = el} position={[-0.55, 0.15, -0.45]} />
            <Wheel ref={el => wheelsRef.current[3] = el} position={[0.55, 0.15, -0.45]} />

            {/* Direction Arrow on top */}
            <DirectionArrow direction={direction} />
        </group>
    );
}

import { forwardRef } from 'react';

const Wheel = forwardRef(function Wheel({ position }, ref) {
    return (
        <group position={position} rotation={[0, 0, Math.PI / 2]}>
            <mesh ref={ref} castShadow>
                <cylinderGeometry args={[0.18, 0.18, 0.12, 24]} />
                <meshStandardMaterial
                    color="#1e293b"
                    metalness={0.4}
                    roughness={0.6}
                />
            </mesh>
            {/* Wheel rim */}
            <mesh>
                <cylinderGeometry args={[0.1, 0.1, 0.13, 16]} />
                <meshStandardMaterial color="#475569" metalness={0.6} roughness={0.4} />
            </mesh>
        </group>
    );
});

function DirectionArrow({ direction }) {
    const arrowRef = useRef();

    useFrame((state) => {
        if (arrowRef.current && direction !== 'stop') {
            arrowRef.current.scale.setScalar(1 + Math.sin(state.clock.elapsedTime * 5) * 0.15);
        }
    });

    if (direction === 'stop') return null;

    const color = direction === 'forward' ? '#22c55e' : '#ef4444';
    const zOffset = direction === 'forward' ? 0.25 : -0.25;
    const rotation = direction === 'forward' ? 0 : Math.PI;

    return (
        <group ref={arrowRef} position={[0, 0.85, zOffset]} rotation={[0, rotation, 0]}>
            <mesh position={[0, 0, -0.08]}>
                <boxGeometry args={[0.06, 0.04, 0.16]} />
                <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.8} />
            </mesh>
            <mesh position={[0, 0, 0.04]} rotation={[Math.PI / 2, 0, 0]}>
                <coneGeometry args={[0.1, 0.12, 3]} />
                <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.8} />
            </mesh>
        </group>
    );
}

export default RobotModel;
