/**
 * WebSocket Bridge Server for Railway Deployment
 * Bridges ESP32 and browser clients
 */

import { WebSocketServer } from 'ws';
import { createServer } from 'http';

// Use Railway's PORT or default to 3001
const PORT = process.env.PORT || 3001;

// Create HTTP server for health checks
const server = createServer((req, res) => {
    if (req.url === '/health') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({
            status: 'ok',
            esp32: esp32Client !== null,
            browsers: browserClients.size
        }));
    } else {
        res.writeHead(200, { 'Content-Type': 'text/plain' });
        res.end('Maze Solver WebSocket Bridge Server');
    }
});

const wss = new WebSocketServer({ server });

let esp32Client = null;
let browserClients = new Set();

console.log(`� WebSocket bridge server starting on port ${PORT}`);

wss.on('connection', (ws, req) => {
    console.log('📥 New connection from:', req.socket.remoteAddress);

    ws.isIdentified = false;
    ws.clientType = null;
    ws.isAlive = true;

    // Heartbeat
    ws.on('pong', () => {
        ws.isAlive = true;
    });

    ws.on('message', (data) => {
        try {
            const message = JSON.parse(data.toString());

            // Handle client identification
            if (message.type === 'identify') {
                ws.isIdentified = true;
                ws.clientType = message.client;

                if (message.client === 'esp32') {
                    esp32Client = ws;
                    console.log('✅ ESP32 connected and identified');

                    // Notify browsers that ESP32 is connected
                    broadcastToBrowsers({ type: 'esp32_status', connected: true });

                } else if (message.client === 'browser') {
                    browserClients.add(ws);
                    console.log(`✅ Browser connected (total: ${browserClients.size})`);

                    // Tell browser if ESP32 is already connected
                    ws.send(JSON.stringify({
                        type: 'esp32_status',
                        connected: esp32Client !== null && esp32Client.readyState === 1
                    }));
                }
                return;
            }

            // Forward messages between clients
            if (ws.clientType === 'esp32') {
                // ESP32 -> All browsers
                broadcastToBrowsers(message);
            } else if (ws.clientType === 'browser') {
                // Browser -> ESP32
                if (esp32Client && esp32Client.readyState === 1) {
                    esp32Client.send(JSON.stringify(message));
                }
            }

        } catch (e) {
            console.error('Failed to parse message:', e.message);
        }
    });

    ws.on('close', () => {
        if (ws === esp32Client) {
            console.log('❌ ESP32 disconnected');
            esp32Client = null;
            broadcastToBrowsers({ type: 'esp32_status', connected: false });
        } else if (browserClients.has(ws)) {
            browserClients.delete(ws);
            console.log(`❌ Browser disconnected (remaining: ${browserClients.size})`);
        }
    });

    ws.on('error', (err) => {
        console.error('WebSocket error:', err.message);
    });
});

function broadcastToBrowsers(message) {
    const data = JSON.stringify(message);
    browserClients.forEach(client => {
        if (client.readyState === 1) {
            client.send(data);
        }
    });
}

// Heartbeat interval to keep connections alive
const heartbeatInterval = setInterval(() => {
    wss.clients.forEach((ws) => {
        if (ws.isAlive === false) {
            return ws.terminate();
        }
        ws.isAlive = false;
        ws.ping();
    });
}, 30000);

wss.on('close', () => {
    clearInterval(heartbeatInterval);
});

// Start server
server.listen(PORT, () => {
    console.log(`✅ Server listening on port ${PORT}`);
    console.log(`🔗 Health check: http://localhost:${PORT}/health`);
});
