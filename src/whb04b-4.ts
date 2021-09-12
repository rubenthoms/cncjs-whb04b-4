#!/usr/bin/env node

import usb from "usb";
import ioClient from "socket.io-client";

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

enum ControllerBuffer {
    unknown = 0,
    FN = 2,
    BUTTON_CODE,
    FEED = 4,
    AXIS = 5,
}

export type ControllerState = {
    axis: Axis | null;
    feed: Feed | null;
    machineState: CncState;
    feedRate: number;
    spindle: number;
};

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

    protected controllerState: ControllerState;
    protected connected: boolean;
    protected kernelDriverActive: boolean;

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

        socket.on("Grbl:state", (state: GrblState) => {
            this.handleGrblData(state);
        });

        this.lastBuffer = null;

        this.deviceHandle = undefined;
        this.interface = undefined;
        this.endpoint = undefined;
        this.connected = false;
        this.kernelDriverActive = false;

        this.controllerState = {
            axis: null,
            feed: null,
            machineState: CncState.IDLE,
            feedRate: 0,
            spindle: 0,
        };
        usb.on("attach", (device: usb.Device) => this.handleAttachedUsbDevice(device));
        usb.on("detach", (device: usb.Device) => this.handleDetachedUsbDevice(device));
        this.connectController();
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
        i++;
        let encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.wpos.x)) / 1000);
        buffer.writeUInt16LE(encoded[0], i);
        buffer.writeUInt16LE(encoded[1], (i += 2));
        encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.wpos.y)) / 1000);
        buffer.writeUInt16LE(encoded[0], (i += 2));
        buffer.writeUInt16LE(encoded[1], (i += 2));
        encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.wpos.z)) / 1000);
        buffer.writeUInt16LE(encoded[0], (i += 2));
        buffer.writeUInt16LE(encoded[1], (i += 2));
        encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.mpos.x)) / 1000);
        buffer.writeUInt16LE(encoded[0], (i += 2));
        buffer.writeUInt16LE(encoded[1], (i += 2));
        encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.mpos.y)) / 1000);
        buffer.writeUInt16LE(encoded[0], (i += 2));
        buffer.writeUInt16LE(encoded[1], (i += 2));
        encoded = this.encodeFloat(Math.round(1000 * parseFloat(this.grblState.status.mpos.z)) / 1000);
        buffer.writeUInt16LE(encoded[0], (i += 2));
        buffer.writeUInt16LE(encoded[1], (i += 2));

        buffer.writeUInt16LE(this.encodeS16(this.controllerState.feedRate), (i += 2));
        buffer.writeUInt16LE(this.encodeS16(this.controllerState.spindle), (i += 2));
        buffer.writeUInt16LE(this.encodeS16(this.grblState.status.feedrate), (i += 2));
        buffer.writeUInt16LE(this.encodeS16(this.grblState.status.spindle), (i += 2));

        buffer[i++] = 0x00;
        buffer[36] = 0;

        console.log(buffer);

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
        if (this.lastBuffer === null || data.compare(this.lastBuffer) !== 0) {
            this.controllerState.axis = data[ControllerBuffer.AXIS];
            this.controllerState.feed = data[ControllerBuffer.FEED];
            const buttonCode = data[ControllerBuffer.BUTTON_CODE];
            const fnButton = data[ControllerBuffer.FN] === 12;
            switch (buttonCode) {
                case Whb04b_4.buttonCodes.reset:
                    if (fnButton) {
                        this.sendCommand("unlock");
                    } else {
                        this.sendCommand("gcode:reset");
                    }
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
                    this.sendGCode(`G28`);
                    break;
            }
        }
        this.lastBuffer = data;
    }

    protected sendCommand(command: string) {
        this.socket.emit("command", this.port, command);
    }

    protected sendGCode(code: string) {
        this.socket.emit("write", this.port, code + "\n");
    }

    protected handleError(error: string) {
        console.log(error);
    }
}

module.exports = {
    Whb04b_4,
};

