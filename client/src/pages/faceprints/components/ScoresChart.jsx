import React from "react";
import { Line } from "react-chartjs-2";

import {
    Chart as ChartJS,
    CategoryScale,
    LinearScale,
    PointElement,
    LineElement,
    Title,
    Tooltip,
    Filler,
} from "chart.js";

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Filler);

const downsample = (arr, maxPoints) => {
    if (arr.length <= maxPoints) return arr;
    const ratio = Math.ceil(arr.length / maxPoints);
    return arr.filter((_, i) => i % ratio === 0);
};

const ScoresChart = ({ times, detectionScores, classificationScores }) => {
    const maxPoints = 999999;//500;

    const sampledTimes = downsample(times, maxPoints).map(t => parseFloat(t.toFixed(1)));
    const sampledDetectionScores = downsample(detectionScores, maxPoints);
    const sampledClassificationScores = downsample(classificationScores, maxPoints);

    const chartData = {
        labels: sampledTimes.map((s) => s + "s"),
        datasets: [
            {
                label: "Detección",
                data: sampledDetectionScores,
                fill: "origin",
                backgroundColor: "rgba(2, 117, 216, 0.15)",
                borderColor: "rgba(2, 117, 216, 1)",
                borderWidth: 3,
                pointRadius: 2,
                pointBackgroundColor: "#ffffff",
            },
            {
                label: "Clasificación",
                data: sampledClassificationScores,
                fill: "origin",
                backgroundColor: "rgba(168, 168, 168, 0.2)",
                borderColor: "rgba(128, 128, 128, 1)",
                borderWidth: 3,
                pointRadius: 2,
                pointBackgroundColor: "rgba(130, 130, 130, 1)",
            },
        ],
    };

    const options = {
        responsive: true,
        maintainAspectRatio: false,
        tension: 0.3,
        plugins: {
            legend: {
                display: true,
                position: "top",
            },
            tooltip: {
                mode: "index",
                intersect: false,
                callbacks: {
                    label: function (context) {
                        const label = context.dataset.label || "";
                        const value = context.raw != null ? context.raw.toFixed(3) : "";
                        return `${label}: ${value}`;
                    },
                },
            },
        },
        scales: {
            x: {
                title: {
                    display: true,
                    text: "Tiempo transcurrido",
                    color: "#444",
                    font: { weight: "bold", size: 14 },
                },
                ticks: {
                    color: "#444",
                    font: { weight: "bold", size: 12 },
                },
                grid: {
                    color: "rgba(255,255,255,0.2)",
                },
            },
            y: {
                beginAtZero: true,
                title: {
                    display: true,
                    text: "Puntuación",
                    color: "#444",
                    font: { weight: "bold", size: 14 },
                },
                ticks: {
                    color: "#444",
                    font: { weight: "bold", size: 12 },
                },
                grid: {
                    color: "rgba(255,255,255,0.2)",
                },
            },
        },
        interaction: {
            mode: "index",
            intersect: false,
        },
    };

    return <Line data={chartData} options={options} />;
};

export default ScoresChart;
