#!/usr/bin/env node

import usb from "usb";
import ioClient from "socket.io-client";
import fs from "fs";
import os from "os";

export enum Axis {
    OFF = 6,
    X = 17,
    Y,
    Z,
    A,
}

export enum Feed {
    MOVE_UNIT_2 = 13,
    MOVE_UNIT_5,
    MOVE_UNIT_10,
    MOVE_UNIT_30,
    MOVE_UNIT_60 = 26,
    MOVE_UNIT_100,
    MOVE_UNIT_LEAD,
}

const FeedStepMap: { [key in Feed]: number } = {
    [Feed.MOVE_UNIT_2]: 0.001,
    [Feed.MOVE_UNIT_5]: 0.01,
    [Feed.MOVE_UNIT_10]: 0.1,
    [Feed.MOVE_UNIT_30]: 1,
    [Feed.MOVE_UNIT_60]: 1,
    [Feed.MOVE_UNIT_100]: 1,
    [Feed.MOVE_UNIT_LEAD]: 1,
};

enum ControllerBuffer {
    unknown = 0,
    BUTTON_CODE = 2,
    FN_BUTTON_CODE = 3,
    FEED = 4,
    AXIS = 5,
    JOG = 6,
}

export type ControllerState = {
    axis: Axis | null;
    feed: Feed | null;
    machineState: CncState;
    feedRate: number;
    spindle: number;
    jogMode: JogMode;
    coordinateSystem: CoordinateSystem;
    distance: Distance;
};

enum JogMode {
    CONTINUOUS = 0,
    STEP,
}

enum CoordinateSystem {
    machineCoordinates = 0,
    workCoordinates = 1,
}

enum CncState {
    IDLE = 0,
    RUN,
    HOLD,
}

type Position = {
    x: string;
    y: string;
    z: string;
};

enum CommandType {
    Position,
    Other,
}

type Command = {
    type: CommandType;
    code: string;
};

interface GrblState {
    parserState: {
        feedRate: string;
        modal: {
            coolant: string;
            distance: string;
            feedrate: string;
            motion: string;
            plane: string;
            spindle: string;
            units: string;
            wcs: string;
        };
        spindle: string;
        tool: string;
    };
    status: {
        activeState: string;
        feedrate: number;
        mpos: Position;
        ov: number[];
        spindle: number;
        subState: 0;
        wco: Position;
        wpos: Position;
    };
}

interface GrblSettings {
    parameters: object;
    settings: {
        [key: string]: string;
    };
    version: string;
}

enum Distance {
    Absolute,
    Relative,
}

const MAX_SPINDLE_SPEED = 25000; //rpm
const MIN_SPINDLE_SPEED = 5000; //rpm
const PROBE_DEPTH = 10; // mm
const TOUCH_PLATE_THICKNESS = 24.2; // mm
const PROBE_FEEDRATE = 20; //mm/min
const RETRACTION_DISTANCE = 4; // mm

const PATH_TO_CNC_RC = os.homedir() + "/.cncrc";

export class Whb04b_4 {
    private static vendorId = 4302;
    private static productId = 60307;
    private static endpointAddress = 129;
    private static buttonCodes = {
        reset: 1,
        stop: 2,
        startPause: 3,
        feedPlus: 4,
        feedMinus: 5,
        spindlePlus: 6,
        spindleMinus: 7,
        machineHome: 8,
        safeZ: 9,
        workingAreaHome: 10,
        spindleOnOff: 11,
        fn: 12,
        probeZ: 13,
        continuous: 14,
        step: 15,
        macro10: 16,
    };

    protected deviceHandle: usb.Device | undefined;
    protected interface: usb.Interface | undefined;
    protected endpoint: usb.Endpoint | undefined;

    protected socket: typeof ioClient.Socket;
    protected port: string;
    protected grblState: GrblState;
    protected grblSettings: GrblSettings;
    protected macros: string[];
    protected cncRcLastModifiedMs: number;

    protected controllerState: ControllerState;
    protected connected: boolean;
    protected kernelDriverActive: boolean;
    protected lastTimestamp: number;
    protected lastVelocity: number;
    protected commandExecuted: boolean;
    protected commandBuffer: Command[];

    private lastBuffer: Buffer | null;

    constructor(socket: typeof ioClient.Socket, port: string) {
        this.socket = socket;
        this.port = port;
        this.grblState = {
            parserState: {
                feedRate: "",
                modal: {
                    coolant: "",
                    distance: "",
                    feedrate: "",
                    motion: "",
                    plane: "",
                    spindle: "",
                    units: "",
                    wcs: "",
                },
                spindle: "",
                tool: "",
            },
            status: {
                activeState: "",
                feedrate: 0,
                mpos: { x: "0", y: "0", z: "0" },
                ov: [0, 0, 0],
                spindle: 0,
                subState: 0,
                wco: { x: "0", y: "0", z: "0" },
                wpos: { x: "0", y: "0", z: "0" },
            },
        };

        this.grblSettings = {
            parameters: {},
            settings: {},
            version: "unknown",
        };

        this.macros = [];
        this.cncRcLastModifiedMs = 0;
        this.maybeReadCncRcFile();
        setInterval(this.maybeReadCncRcFile, 5000);

        socket.on("Grbl:state", (state: GrblState) => {
            this.handleGrblData(state);
        });

        socket.on("Grbl:settings", (state: GrblSettings) => {
            this.handleGrblSettings(state);
        });

        socket.on("serialport:read", (data: string) => {
            if ((data || "").trim() === "ok") {
                this.commandExecuted = true;
                this.processCommandBuffer();
            }
        });

        this.lastBuffer = null;
        this.lastTimestamp = Date.now();
        this.lastVelocity = 0;

        this.deviceHandle = undefined;
        this.interface = undefined;
        this.endpoint = undefined;
        this.connected = false;
        this.kernelDriverActive = false;
        this.commandExecuted = true;
        this.commandBuffer = [];

        this.controllerState = {
            axis: null,
            feed: null,
            machineState: CncState.IDLE,
            feedRate: 0,
            spindle: 5000,
            jogMode: JogMode.CONTINUOUS,
            coordinateSystem: CoordinateSystem.machineCoordinates,
            distance: Distance.Absolute,
        };
        usb.on("attach", (device: usb.Device) => this.handleAttachedUsbDevice(device));
        usb.on("detach", (device: usb.Device) => this.handleDetachedUsbDevice(device));
        this.connectController();
    }

    protected maybeReadCncRcFile(): void {
        fs.open(PATH_TO_CNC_RC, "r", (err, _) => {
            if (err) {
                console.log(err);
                return;
            }

            fs.stat(PATH_TO_CNC_RC, (_, data) => {
                if (data.mtimeMs > this.cncRcLastModifiedMs) {
                    this.macros = [];
                    const rawData = fs.readFileSync(PATH_TO_CNC_RC, "utf-8");
                    const jsonData = JSON.parse(rawData);
                    if ("macros" in jsonData && Array.isArray(jsonData["macros"])) {
                        jsonData["macros"].forEach((macro) => {
                            if ("id" in macro) {
                                this.macros.push(macro["id"]);
                            }
                        });
                    }
                    this.cncRcLastModifiedMs = data.mtimeMs;
                }
            });
        });
    }

    protected handleAttachedUsbDevice(device: usb.Device): void {
        if (this.connected) {
            return;
        }
        if (
            device.deviceDescriptor.idProduct === Whb04b_4.productId &&
            device.deviceDescriptor.idVendor === Whb04b_4.vendorId
        ) {
            this.connectController();
        }
    }

    protected handleDetachedUsbDevice(device: usb.Device): void {
        if (!this.connected) {
            return;
        }
        if (
            device.deviceDescriptor.idProduct === Whb04b_4.productId &&
            device.deviceDescriptor.idVendor === Whb04b_4.vendorId
        ) {
            this.connected = false;
            console.log("Controller disconnected");
        }
    }

    protected connectController(): void {
        const devices = usb.getDeviceList();
        if (devices.length < 0) {
            console.log("Error!");
        }

        this.deviceHandle = usb.findByIds(Whb04b_4.vendorId, Whb04b_4.productId);
        if (!this.deviceHandle) {
            this.connected = false;
            return;
        }

        this.deviceHandle.open();
        this.interface = this.deviceHandle.interface(0);
        this.kernelDriverActive = this.interface.isKernelDriverActive();
        if (this.kernelDriverActive) {
            this.interface.detachKernelDriver();
        }
        this.interface.claim();

        this.endpoint = this.interface.endpoint(Whb04b_4.endpointAddress);

        if (this.endpoint.direction !== "in") {
            this.disconnectController();
            return;
        }

        this.endpoint.on("data", (data: Buffer) => this.handleData(data));
        this.endpoint.on("error", (error: string) => this.handleError(error));

        (this.endpoint as usb.InEndpoint).startPoll(1, 1000);

        console.log("Controller connected!");
        this.connected = true;
        setTimeout(this.displayEncode, 1000);
    }

    protected disconnectController(): void {
        if (this.endpoint) {
            (this.endpoint as usb.InEndpoint).stopPoll();
            this.endpoint.off("data", this.handleData);
            this.endpoint.off("error", this.handleError);
        }
        if (this.interface) {
            this.interface.release();
            if (this.kernelDriverActive) {
                this.interface.attachKernelDriver();
            }
        }
        if (this.deviceHandle) {
            this.deviceHandle.close();
        }
        this.connected = false;
        console.log("Controller disconnected");
    }

    protected handleGrblData(state: GrblState) {
        this.grblState = state;
        if (state.status.activeState === "Run") {
            this.controllerState.machineState = CncState.RUN;
        } else if (state.status.activeState === "Hold") {
            this.controllerState.machineState = CncState.HOLD;
        } else if (state.status.activeState === "Idle") {
            this.controllerState.machineState = CncState.IDLE;
        }
        this.displayEncode();
    }

    protected handleGrblSettings(settings: GrblSettings) {
        this.grblSettings = settings;
    }

    protected dec2bin(v: number, numBits: number = 16) {
        const bin = (v >>> 0).toString(2);
        return bin.substring(bin.length - numBits - 1, bin.length - 1);
    }

    protected encodeFloat(v: number): number[] {
        const intV = Math.round(Math.abs(v) * 10000.0);
        const intPart = ((intV / 10000) << 16) >> 16;
        let fractPart = (intV % 10000 << 16) >> 16;
        if (v < 0) fractPart = fractPart | 0x8000;
        return [Math.floor(intPart), fractPart];
    }

    protected encodeS16(v: number) {
        return (v << 16) >> 16;
    }

    protected setBit(buffer: Buffer, byte: number, bit: number, value: number) {
        if (value === 0) {
            buffer[byte] &= ~(1 << bit);
        } else {
            buffer[byte] |= 1 << bit;
        }
    }

    protected displayEncode() {
        if (!this.deviceHandle) {
            return;
        }

        const buffer = Buffer.alloc(6 * 7, 0);
        const data = Buffer.alloc(6 * 8, 0);

        let i = 0;
        buffer[i++] = 0xfe;
        buffer[i++] = 0xfd;
        buffer[i++] = 0x0c;
        buffer[i] = 0x0;

        this.setBit(buffer, i, 0, this.controllerState.jogMode === JogMode.CONTINUOUS ? 0 : 1);
        this.setBit(buffer, i, 7, this.controllerState.coordinateSystem === CoordinateSystem.workCoordinates ? 1 : 0);
        i++;
        let encoded = [0, 0];
        if (this.controllerState.coordinateSystem === CoordinateSystem.workCoordinates) {
            encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.wpos.x)) / 1000);
            buffer.writeUInt16LE(encoded[0], i);
            buffer.writeUInt16LE(encoded[1], (i += 2));
            encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.wpos.y)) / 1000);
            buffer.writeUInt16LE(encoded[0], (i += 2));
            buffer.writeUInt16LE(encoded[1], (i += 2));
            encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.wpos.z)) / 1000);
            buffer.writeUInt16LE(encoded[0], (i += 2));
            buffer.writeUInt16LE(encoded[1], (i += 2));
        } else {
            encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.mpos.x)) / 1000);
            buffer.writeUInt16LE(encoded[0], i);
            buffer.writeUInt16LE(encoded[1], (i += 2));
            encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.mpos.y)) / 1000);
            buffer.writeUInt16LE(encoded[0], (i += 2));
            buffer.writeUInt16LE(encoded[1], (i += 2));
            encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.mpos.z)) / 1000);
            buffer.writeUInt16LE(encoded[0], (i += 2));
            buffer.writeUInt16LE(encoded[1], (i += 2));
        }
        i++;
        buffer.writeUInt16LE(this.controllerState.feedRate, (i += 2));
        buffer.writeUInt16LE(this.controllerState.spindle, (i += 2));
        buffer[i++] = 0x00;
        buffer[36] = 0;

        i = 0;
        for (let packageIndex = 0; packageIndex < 6; packageIndex++) {
            for (let j = 0; j < 8; j++) {
                if (j === 0) {
                    data[j + 8 * packageIndex] = 6;
                } else {
                    data[j + 8 * packageIndex] = buffer[i++];
                }
            }
        }

        for (let packageIndex = 0; packageIndex < 6; packageIndex++) {
            this.deviceHandle.controlTransfer(
                0x21,
                0x09,
                0x0306,
                0x00,
                data.slice(8 * packageIndex, 8 * (packageIndex + 1)),
                (error?: usb.LibUSBException) => error && this.connectController()
            );
        }
    }

    protected handleData(data: Buffer) {
        const newTimestamp = Date.now();
        const timeElapsedMs = newTimestamp - this.lastTimestamp;
        this.controllerState.axis = data[ControllerBuffer.AXIS];
        this.controllerState.feed = data[ControllerBuffer.FEED];
        const buttonCode = data[ControllerBuffer.BUTTON_CODE];
        const fnButtonCode = data[ControllerBuffer.FN_BUTTON_CODE];
        if (this.lastBuffer === null || data.compare(this.lastBuffer) !== 0) {
            if (buttonCode !== 12) {
                switch (buttonCode) {
                    case Whb04b_4.buttonCodes.reset:
                        this.sendCommand("gcode:reset");
                        break;
                    case Whb04b_4.buttonCodes.stop:
                        this.sendCommand("gcode:stop");
                        break;
                    case Whb04b_4.buttonCodes.startPause:
                        if (this.controllerState.machineState === CncState.RUN) {
                            this.sendCommand("gcode:pause");
                        } else if (this.controllerState.machineState === CncState.HOLD) {
                            this.sendCommand("gcode:resume");
                        } else {
                            this.sendCommand("gcode:start");
                        }
                        break;
                    case Whb04b_4.buttonCodes.machineHome:
                        this.sendCommand("homing");
                        break;
                    case Whb04b_4.buttonCodes.spindleOnOff:
                        if (this.grblState.status.spindle === 0) {
                            this.sendGCode(`M03 S${this.controllerState.spindle}`);
                        } else {
                            this.sendGCode("M05");
                        }
                        break;
                    case Whb04b_4.buttonCodes.workingAreaHome:
                        if (this.controllerState.coordinateSystem === CoordinateSystem.machineCoordinates) {
                            this.sendGCode(`G54 G90 G0 X0 Y0 Z0`);
                        } else {
                            this.sendGCode(`G54 G90 G0 X0 Y0 Z0`);
                        }
                        break;
                    case Whb04b_4.buttonCodes.probeZ:
                        this.sendGCode(`G91`);
                        this.sendGCode(`G38.2 Z-${PROBE_DEPTH} F${PROBE_FEEDRATE}`);
                        this.sendGCode(`G90`);
                        this.sendGCode(`G10 L20 P1 Z${TOUCH_PLATE_THICKNESS}`);
                        this.sendGCode(`G91`);
                        this.sendGCode(`G0 Z${RETRACTION_DISTANCE}`);
                        this.sendGCode(`G90`);
                        break;
                    case Whb04b_4.buttonCodes.spindlePlus:
                        this.controllerState.spindle = Math.min(this.controllerState.spindle + 500, MAX_SPINDLE_SPEED);
                        this.sendGCode(`S${this.controllerState.spindle}`);
                        break;
                    case Whb04b_4.buttonCodes.spindleMinus:
                        this.controllerState.spindle = Math.max(this.controllerState.spindle - 500, MIN_SPINDLE_SPEED);
                        this.sendGCode(`S${this.controllerState.spindle}`);
                        break;
                    case Whb04b_4.buttonCodes.safeZ:
                        this.sendGCode(`G53 Z0`);
                        break;
                    case Whb04b_4.buttonCodes.continuous:
                        this.controllerState.jogMode = JogMode.CONTINUOUS;
                        this.displayEncode();
                        break;
                    case Whb04b_4.buttonCodes.step:
                        this.controllerState.jogMode = JogMode.STEP;
                        this.displayEncode();
                        break;
                    case Whb04b_4.buttonCodes.macro10:
                        if (this.controllerState.coordinateSystem === CoordinateSystem.machineCoordinates) {
                            this.controllerState.coordinateSystem = CoordinateSystem.workCoordinates;
                        } else {
                            this.controllerState.coordinateSystem = CoordinateSystem.machineCoordinates;
                        }
                        this.displayEncode();
                        break;
                }
            } else {
                switch (fnButtonCode) {
                    case Whb04b_4.buttonCodes.reset:
                        this.sendCommand("unlock");
                        break;
                    case Whb04b_4.buttonCodes.feedPlus:
                        this.runMacro(0);
                        break;
                    case Whb04b_4.buttonCodes.feedMinus:
                        this.runMacro(1);
                        break;
                    case Whb04b_4.buttonCodes.spindlePlus:
                        this.runMacro(2);
                        break;
                    case Whb04b_4.buttonCodes.spindleMinus:
                        this.runMacro(3);
                        break;
                    case Whb04b_4.buttonCodes.machineHome:
                        this.runMacro(4);
                        break;
                    case Whb04b_4.buttonCodes.safeZ:
                        this.runMacro(5);
                        break;
                    case Whb04b_4.buttonCodes.workingAreaHome:
                        this.runMacro(6);
                        break;
                    case Whb04b_4.buttonCodes.spindleOnOff:
                        this.runMacro(7);
                        break;
                    case Whb04b_4.buttonCodes.probeZ:
                        this.runMacro(8);
                        break;
                }
            }
        }
        const jogCounts =
            data[ControllerBuffer.JOG] > 128 ? data[ControllerBuffer.JOG] - 256 : data[ControllerBuffer.JOG];

        if (this.controllerState.jogMode === JogMode.STEP && Math.abs(jogCounts) > 0) {
            const steps = Math.round(jogCounts * FeedStepMap[this.controllerState.feed] * 1000) / 1000;
            switch (this.controllerState.axis) {
                case Axis.X:
                    this.sendPositionGCode(`G0 X${steps}`);
                    break;
                case Axis.Y:
                    this.sendPositionGCode(`G0 Y${steps}`);
                    break;
                case Axis.Z:
                    this.sendPositionGCode(`G0 Z${steps}`);
                    break;
                case Axis.A:
                    this.sendPositionGCode(`G0 A${steps}`);
                    break;
            }
        } else if (this.controllerState.jogMode === JogMode.CONTINUOUS && Math.abs(jogCounts) > 0) {
            this.commandBuffer = [];
            let k = 0; // %
            switch (this.controllerState.feed) {
                case Feed.MOVE_UNIT_2:
                    k = 0.02;
                    break;
                case Feed.MOVE_UNIT_10:
                    k = 0.1;
                    break;
                case Feed.MOVE_UNIT_30:
                    k = 0.3;
                    break;
                case Feed.MOVE_UNIT_60:
                    k = 0.6;
                    break;
                case Feed.MOVE_UNIT_100:
                    k = 1.0;
                    break;
            }
            let maxVelocity = 0; // mm/min
            let maxAcceleration = 0; // mm/s*s
            switch (this.controllerState.axis) {
                case Axis.X:
                    maxVelocity = parseFloat(this.grblSettings.settings["$110"]);
                    maxAcceleration = parseFloat(this.grblSettings.settings["$120"]);
                    break;
                case Axis.Y:
                    maxVelocity = parseFloat(this.grblSettings.settings["$111"]);
                    maxAcceleration = parseFloat(this.grblSettings.settings["$121"]);
                    break;
                case Axis.Z:
                    maxVelocity = parseFloat(this.grblSettings.settings["$112"]);
                    maxAcceleration = parseFloat(this.grblSettings.settings["$122"]);
                    break;
            }
            const requestedVelocity = Math.max(maxVelocity, k * maxVelocity * Math.abs(jogCounts));
            const actualVelocity = Math.max(
                requestedVelocity,
                this.lastVelocity + (maxAcceleration * timeElapsedMs) / 1000
            );
            const distance =
                (Math.sign(jogCounts) * Math.round(((actualVelocity * timeElapsedMs) / 60000) * 1000)) / 1000;

            if (this.commandExecuted) {
                switch (this.controllerState.axis) {
                    case Axis.X:
                        this.sendPositionGCode(`G0 X${distance}`);
                        break;
                    case Axis.Y:
                        this.sendPositionGCode(`G0 Y${distance}`);
                        break;
                    case Axis.Z:
                        this.sendPositionGCode(`G0 Z${distance}`);
                        break;
                }
            }
        } else {
            let maxAcceleration = 0; // mm/s*s
            switch (this.controllerState.axis) {
                case Axis.X:
                    maxAcceleration = parseFloat(this.grblSettings.settings["$120"]);
                    break;
                case Axis.Y:
                    maxAcceleration = parseFloat(this.grblSettings.settings["$121"]);
                    break;
                case Axis.Z:
                    maxAcceleration = parseFloat(this.grblSettings.settings["$122"]);
                    break;
            }
            this.lastVelocity = Math.max(0, this.lastVelocity - (maxAcceleration * timeElapsedMs) / 1000);
            this.commandBuffer = [];
        }
        this.lastBuffer = data;
        this.lastTimestamp = newTimestamp;
    }

    protected runMacro(macroIndex: number) {
        if (macroIndex >= 0 && macroIndex < this.macros.length) {
            const macroId = this.macros[macroIndex];
            this.socket.emit("command", this.port, "macro:load", macroId);
            this.socket.emit("command", this.port, "macro:run", macroId);
        }
    }

    protected sendCommand(command: string) {
        this.socket.emit("command", this.port, command);
    }

    protected sendGCode(code: string) {
        this.socket.emit("write", this.port, code + "\n");
    }

    protected sendPositionGCode(code: string) {
        this.commandBuffer.push({ type: CommandType.Position, code: code });
        this.processCommandBuffer();
    }

    protected processCommandBuffer() {
        if (this.commandBuffer.length > 0 && this.commandExecuted) {
            if (
                this.commandBuffer[0].type === CommandType.Position &&
                this.controllerState.distance === Distance.Absolute
            ) {
                this.controllerState.distance = Distance.Relative;
                this.commandBuffer.unshift({ type: CommandType.Other, code: "G91" });
            } else if (
                this.commandBuffer[0].type !== CommandType.Position &&
                this.controllerState.distance === Distance.Relative
            ) {
                this.controllerState.distance = Distance.Absolute;
                this.commandBuffer.unshift({ type: CommandType.Other, code: "G90" });
            }
            this.commandExecuted = false;
            this.socket.emit("write", this.port, this.commandBuffer[0].code + "\n");
            this.commandBuffer = this.commandBuffer.slice(1, this.commandBuffer.length);
        } else if (
            this.commandBuffer.length === 0 &&
            this.commandExecuted &&
            this.controllerState.distance === Distance.Relative
        ) {
            this.controllerState.distance = Distance.Absolute;
            this.socket.emit("write", this.port, "G90\n");
        }
    }

    protected handleError(error: string) {
        console.log(error);
    }
}

module.exports = {
    Whb04b_4,
};
