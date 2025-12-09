/**
 * HUD Overlay - White Modern Theme
 */
export function HUD({ sensorData }) {
    const { yaw, direction } = sensorData;

    return (
        <>
            {/* Compass - Top Right */}
            <div className="absolute top-20 right-4 z-10">
                <div className="w-20 h-20 rounded-full bg-white/90 border border-slate-200 shadow-lg flex items-center justify-center backdrop-blur-sm">
                    <svg
                        width="60"
                        height="60"
                        viewBox="0 0 60 60"
                        style={{ transform: `rotate(${-yaw}deg)`, transition: 'transform 0.1s ease-out' }}
                    >
                        {/* Compass ring */}
                        <circle cx="30" cy="30" r="28" fill="none" stroke="#e2e8f0" strokeWidth="1" />

                        {/* Cardinal marks */}
                        <text x="30" y="10" textAnchor="middle" fontSize="8" fill="#3b82f6" fontWeight="bold">N</text>
                        <text x="54" y="33" textAnchor="middle" fontSize="7" fill="#94a3b8">E</text>
                        <text x="30" y="56" textAnchor="middle" fontSize="7" fill="#94a3b8">S</text>
                        <text x="6" y="33" textAnchor="middle" fontSize="7" fill="#94a3b8">W</text>

                        {/* Needle */}
                        <polygon points="30,12 33,30 30,35 27,30" fill="#3b82f6" />
                        <polygon points="30,48 33,30 30,25 27,30" fill="#cbd5e1" />

                        {/* Center dot */}
                        <circle cx="30" cy="30" r="3" fill="#3b82f6" />
                    </svg>
                </div>
            </div>

            {/* Direction Indicator - Bottom Right */}
            <div className="absolute bottom-6 right-4 z-10">
                <div className={`px-4 py-2 rounded-full font-bold text-sm shadow-lg backdrop-blur-sm border ${direction === 'forward'
                        ? 'bg-emerald-50 text-emerald-600 border-emerald-200'
                        : direction === 'backward'
                            ? 'bg-rose-50 text-rose-600 border-rose-200'
                            : 'bg-white text-slate-400 border-slate-200'
                    }`}>
                    {direction === 'forward' && <span>↑ FWD</span>}
                    {direction === 'backward' && <span>↓ REV</span>}
                    {direction === 'stop' && <span>— STOP</span>}
                </div>
            </div>

            {/* Heading Value - Bottom Left */}
            <div className="absolute bottom-6 left-4 z-10">
                <div className="bg-white/90 backdrop-blur-sm border border-slate-200 rounded-xl px-4 py-3 shadow-lg">
                    <div className="text-[10px] text-slate-400 uppercase tracking-wider mb-1">Heading</div>
                    <div className="text-2xl font-mono font-bold text-blue-600 tabular-nums">
                        {yaw.toFixed(1)}°
                    </div>
                </div>
            </div>
        </>
    );
}

export default HUD;
