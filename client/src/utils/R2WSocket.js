const R2W_MESSAGE_TYPE = {
    MESSAGE: "MESSAGE",
    TOPIC: "TOPIC",
};

export default class R2WSocket {
    constructor(url) {
        this.socket = new WebSocket(url); // Usar reconnecting websocket

        this.socket.onopen = (event) => {
            if (this.onopen) this.onopen(event);
        };

        this.socket.onmessage = (event) => {
            let r2w_message;
            try {
                r2w_message = JSON.parse(event.data);
            } catch (e) {
                console.log("ERROR ON JSON PARSE ON R2WSocket", e);
                console.log("ORIGINAL MESSAGE", event.data);
            }            
            
            if (r2w_message.type === R2W_MESSAGE_TYPE.MESSAGE) {
                const message = JSON.parse(r2w_message.data); // Ver si cambiar esto para que ya venga parseado del nodo ros
                if (this.onmessage) this.onmessage(message);
            } else if  (r2w_message.type === R2W_MESSAGE_TYPE.TOPIC) {
                const message = r2w_message.data;
                if (this.ontopic) this.ontopic(message);
            }
        };

        this.socket.onclose = (event) => {
            if (this.onclose) this.onclose(event);
        };

        this.socket.onerror = (error) => {
            if (this.onerror) this.onerror(error);
        };
    }

    send(message) {
        const messageR2W = {
            type: "MESSAGE",
            data: message,
        };

        this.socket.send(JSON.stringify(messageR2W));
    }

    close() {
        this.socket.close();
    }
}
