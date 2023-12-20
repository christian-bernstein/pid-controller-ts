export type PIDGains = {
    p: number,
    i: number,
    d: number
}

export type PIDOutput = {
    min: number,
    max: number,
    minThreshold: number
}

export type PIDITermLimit = {
    min: number,
    max: number
}

export class PIDController {

    private _target?: number;

    private gains: PIDGains = {
        p: 0,
        i: 0,
        d: 0
    };

    private iTerm = 0;

    private iTermLimit: PIDITermLimit = {
        min: -100,
        max: 100
    }

    private previousError = 0;

    private output: PIDOutput = {
        min: -100,
        max: 100,
        minThreshold: 0
    };

    private _tolerance: number = 0;

    public setTarget(target: number = 0) {
        this._target = target;
    }

    public setGains(p: number = 0, d: number = 0, i: number = 0) {
        this.gains = {
            p: p,
            d: d,
            i: i
        };
    }

    public setOutputs(min: number = 0, max: number = 0, minThreshold: number = 0) {
        this.output = {
            min: min,
            max: max,
            minThreshold: minThreshold
        };
    }

    public setOutputBounds(min: number, max: number) {
        this.output = {
            min: min,
            max: max,
            minThreshold: this.output?.minThreshold ?? 0
        };
    }

    public setItermLimit(min: number = 0, max: number = 0) {
        this.iTermLimit = {
            min: min,
            max: max
        };
    }

    public run(current: number = 0): number {

        let exactError = current - (this._target ?? current);

        let error = Math.abs(exactError) > this.tolerance ? exactError : 0,
            pTerm = error * this.gains.p,
            dTerm = (error - this.previousError) * this.gains.d,
            output;

        this.previousError = error;

        this.iTerm += error * this.gains.i;
        if (this.iTerm > this.iTermLimit.max) {
            this.iTerm = this.iTermLimit.max;
        } else if (this.iTerm < this.iTermLimit.min) {
            this.iTerm = this.iTermLimit.min;
        }

        output = pTerm + this.iTerm + dTerm;
        if (output < this.output.minThreshold) {
            output = this.output.min;
        } else if (output > this.output.max) {
            output = this.output.max;
        }

        return output;
    }

    set tolerance(value: number) {
        if (value < 0) throw new Error(`PID controller tolerance cannot be set to a negative value: '${value}'`);
        this._tolerance = value;
    }

    get tolerance(): number {
        return this._tolerance;
    }

    get error(): number {
        return this.previousError;
    }

    get target(): number {
        return this._target ?? 0;
    }

    get outputMax(): number {
        return this.output.max;
    }
}
