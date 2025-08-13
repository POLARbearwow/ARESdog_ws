const ip = '192.168.31.248:'
const port = '9090'; // 替换为你的Rosbridge服务器端口
async function loadConfig() {
    try {
        const response = await fetch('config.json');
        if (!response.ok) throw new Error('Failed to load config file');
        const config_data = await response.json();
        console.log('Config file loaded successfully:', config_data);
        if (!config_data.rosbridge_ip || !config_data.rosbridge_port || !config_data.http_port) {
            throw new Error('Necessary configuration fields are missing');
        }
        return config_data;
    } catch (error) {
        console.error('Error while loading config:', error);
        // 提供默认配置作为回退
        return {
            rosbridge_ip: "192.168.31.248",
            rosbridge_port: "9090",
            http_port: 8080,
            publish_interval: 10,
            dead_zone: 0.2,
            _isFallBack: true
        };
    }
}

// 全局变量


// 摇杆输出值
let leftOutput = { x: 0, y: 0 };
let rightOutput = { x: 0 };

// 活动触摸点记录
let activeTouches = {
    left: null,
    vertical: null,
    right: null,
    pitch: null,
    gaitPeriod: null,
    gaitLength: null,
    ZSwing: null
};

// 高度滑块控制元素
const verticalSliderKnob = document.getElementById('vertical-slider-knob');
const verticalSliderContainer = document.querySelector('.vertical-slider-container');
let verticalValue = 0.5;
let verticalActive = false;
let verticalStartY = 0;
let verticalSliderHeight = 200;

function initVerticalSlider() {
    updateVerticalSliderPostion(verticalValue);
}

const pitchSliderKnob = document.getElementById('pitch-slider-knob');
const pitchSliderContainer = document.querySelector('.pitch-slider-container');
let pitchValue = 0.5;
let pitchActive = false;
let pitchStartY = 0;
let pitchSliderHeight = 200;

function initPitchSlider() {
    updatePitchSliderPosition(pitchValue);
}

// 贝塞尔曲线变换函数
function applyVerticalCurveTransform(progress) {
    return getPointOnBezierCurve(
        progress,
        verticalBezierCurvePoints.p0,
        verticalBezierCurvePoints.p1,
        verticalBezierCurvePoints.p2,
        verticalBezierCurvePoints.p3
    );
}
function applyPitchCurveTransform(progress) {
    return getPointOnBezierCurve(
        progress,
        pitchBezierCurvePoints.p0,
        pitchBezierCurvePoints.p1,
        pitchBezierCurvePoints.p2,
        pitchBezierCurvePoints.p3
    );
}
const verticalBezierCurvePoints = {
    p0: {x: 200, y: 200},
    p1: {x: 180, y: 100},
    p2: {x: 75, y: 35},
    p3: {x: 25, y: 25}
};
const pitchBezierCurvePoints = {
    p0: {x: 55, y: 200},
    p1: {x: 75, y: 100},
    p2: {x: 180, y: 35},
    p3: {x: 230, y: 25}
}

function getPointOnBezierCurve(percent, p0, p1, p2, p3) {
    // 确保百分比在0-1之间
    const t = Math.max(0, Math.min(1, percent));
    
    // 三次贝塞尔曲线公式
    const x = Math.pow(1 - t, 3) * p0.x + 
              3 * Math.pow(1 - t, 2) * t * p1.x + 
              3 * (1 - t) * Math.pow(t, 2) * p2.x + 
              Math.pow(t, 3) * p3.x;
              
    const y = Math.pow(1 - t, 3) * p0.y + 
              3 * Math.pow(1 - t, 2) * t * p1.y + 
              3 * (1 - t) * Math.pow(t, 2) * p2.y + 
              Math.pow(t, 3) * p3.y;
    
    return { x, y };
}

function updateVerticalSliderPostion(progress) {
    verticalValue = progress;
    document.getElementById('vertical-value').textContent = verticalValue.toFixed(2);
    console.info(`Updating vertical slider position: ${verticalValue}`);
    const knobPosition = applyVerticalCurveTransform(progress);

    verticalSliderKnob.style.left = `${knobPosition.x}px`;
    verticalSliderKnob.style.top = `${knobPosition.y}px`;

    updateVerticalFillPath(progress);
}

function updatePitchSliderPosition(progress) {
    pitchValue = progress;
    document.getElementById('pitch-value').textContent = pitchValue.toFixed(2);
    console.info(`Updating pitch slider position: ${pitchValue}`);
    const knobPosition = applyPitchCurveTransform(progress);

    pitchSliderKnob.style.left = `${knobPosition.x}px`;
    pitchSliderKnob.style.top = `${knobPosition.y}px`;

    updatePitchFillPath(progress);
}

function updateVerticalFillPath(progress) {
    const path = document.getElementById('vertical-fill-path');
    const segments = 20;

    let d = `M200, 200`;
    for (let i = 0; i <= segments; i++) {
        const p = Math.min(i / segments, progress);
        const point = applyVerticalCurveTransform(p);
        d += ` L${point.x},${point.y}`;
    }

    path.setAttribute('d', d);
}

function updatePitchFillPath(progress) {
    const path = document.getElementById('pitch-fill-path');
    const segments = 20;

    let d = `M55, 200`;
    for (let i = 0; i <= segments; i++) {
        const p = Math.min(i / segments, progress);
        const point = applyPitchCurveTransform(p);
        d += ` L${point.x},${point.y}`;
    }

    path.setAttribute('d', d);
}

verticalSliderContainer.addEventListener('touchstart', (event) => {
    if (activeTouches.vertical !== null) return;
    event.preventDefault();
    const touch = event.changedTouches[0];
    verticalActive = true;
    activeTouches.vertical = touch.identifier;
    updateVerticalSliderFromEvent(event);
    updateDebugInfo("height-info", "Height slider touch start");
});
verticalSliderKnob.addEventListener('touchstart', startDragVertical);
verticalSliderContainer.addEventListener('mousedown', startDragVertical);

function startDragVertical(event) {
    if (activeTouches.vertical !== null) return;
    event.preventDefault();
    verticalActive = true;
    if (event.touches) {
        for (let i = 0; i < event.touches.length; i++) {
            if (event.touches[i].target === verticalSliderKnob || event.touches[i].target === verticalSliderContainer) {
                activeTouches.vertical = event.touches[i].identifier;
                break;
            }
        }
    } else {
        activeTouches.vertical = 'mouse';
    }
    updateVerticalSliderFromEvent(event);
    updateDebugInfo("height-info", "Height slider touch start");
}

pitchSliderContainer.addEventListener('touchstart', (event) => {
    if (activeTouches.pitch !== null) return;
    event.preventDefault();
    const touch = event.changedTouches[0];
    pitchActive = true;
    activeTouches.pitch = touch.identifier;
    updatePitchSliderFromEvent(event);
    updateDebugInfo("pitch-info", "Pitch slider touch start");
});
pitchSliderKnob.addEventListener('touchstart', startDragPitch);
pitchSliderContainer.addEventListener('mousedown', startDragPitch);

function startDragPitch(event) {
    if (activeTouches.pitch !== null) return;
    event.preventDefault();
    pitchActive = true;
    if (event.touches) {
        for (let i = 0; i < event.touches.length; i++) {
            if (event.touches[i].target === pitchSliderKnob || event.touches[i].target === pitchSliderContainer) {
                activeTouches.pitch = event.touches[i].identifier;
                break;
            }
        }
    }
    else {
        activeTouches.pitch = 'mouse';
    }
    updatePitchSliderFromEvent(event);
    updateDebugInfo("pitch-info", "Pitch slider touch start");
}

function updateVerticalSliderFromEvent(event) {
    const rect = verticalSliderContainer.getBoundingClientRect();
    let mouseX, mouseY;

    if (event.type == 'touchmove'){
        for (let i = 0; i < event.touches.length; i++) {
            if (event.touches[i].identifier === activeTouches.vertical) {
                mouseX = event.touches[i].clientX;
                mouseY = event.touches[i].clientY;
                break;
            }
        }
    } else {
        mouseX = event.clientX;
        mouseY = event.clientY;
    }

    if (!mouseX || !mouseY) {
        console.warn("Mouse coordinates not found, aborting update.");
        return;
    }

    const x = mouseX - rect.left;
    const y = mouseY - rect.top;

    let closestProgress = 0;
    let minDistance = Infinity;

    for (let p = 0; p < 1.01; p+=0.01){
        const point = applyVerticalCurveTransform(p);
        const distance = Math.sqrt(Math.pow(point.x - x, 2) + Math.pow(point.y - y, 2));

        if (distance < minDistance) {
            minDistance = distance;
            closestProgress = p;
        }
    }

    verticalValue = closestProgress;
    onParamsChanged();
    updateVerticalSliderPostion(verticalValue);
    updateDebugInfo("height-info", `Height slider updated: ${verticalValue.toFixed(2)}`);
}

function updatePitchSliderFromEvent(event) {
    const rect = pitchSliderContainer.getBoundingClientRect();
    let mouseX, mouseY;

    if (event.type == 'touchmove'){
        for (let i = 0; i < event.touches.length; i++) {
            if (event.touches[i].identifier === activeTouches.pitch) {
                mouseX = event.touches[i].clientX;
                mouseY = event.touches[i].clientY;
                break;
            }
        }
    } else {
        mouseX = event.clientX;
        mouseY = event.clientY;
    }

    if (!mouseX || !mouseY) {
        console.warn("Mouse coordinates not found, aborting update.");
        return;
    }

    const x = mouseX - rect.left
    const y = mouseY - rect.top;
    let closestProgress = 0;
    let minDistance = Infinity;
    for (let p = 0; p < 1.01; p+=0.01){
        const point = applyPitchCurveTransform(p);
        const distance = Math.sqrt(Math.pow(point.x - x, 2) + Math.pow(point.y - y, 2));

        if (distance < minDistance) {
            minDistance = distance;
            closestProgress = p;
        }
    }
    pitchValue = closestProgress;
    onParamsChanged();
    updatePitchSliderPosition(pitchValue);
    updateDebugInfo("pitch-info", `Pitch slider updated: ${pitchValue.toFixed(2)}`);
}

const gaitPeriodContainer = document.querySelector('.gait-period-container');
const gaitPeriodSVG = document.getElementById('gait-period-svg');
const gaitPeriodTrackPath = document.getElementById('gait-period-track-path');
const gaitPeriodFillPath = document.getElementById('gait-period-fill-path');
const gaitPeriodKnob = document.getElementById('gait-period-knob');
let gaitPeriodValue = 0.3;
let gaitPeriodActive = false;
const gaitPeriodStartY = 180;
let gaitPeriodSliderHeight = 160;
function initGaitPeriodSlider() {
    updateGaitPeriodSliderPosition(gaitPeriodValue);
}

function updateGaitPeriodSliderPosition(progress) {
    gaitPeriodValue = progress;
    document.getElementById('gait-period-value').textContent = gaitPeriodValue.toFixed(2);
    console.info(`Updating gait period slider position: ${gaitPeriodValue}`);
    const knobPosition = gaitPeriodStartY - progress * gaitPeriodSliderHeight;

    gaitPeriodKnob.style.top = `${knobPosition}px`;

    updateGaitPeriodFillPath(progress);
}
function updateGaitPeriodFillPath(progress) {
    const path = document.getElementById('gait-period-fill-path');
    const segments = 20;

    let d = `M 50, ${gaitPeriodStartY}`;
    for (let i = 0; i <= segments; i++) {
        const p = Math.min(i / segments, progress);
        const y = gaitPeriodStartY - p * gaitPeriodSliderHeight;
        d += ` L${50},${y}`;
    }

    path.setAttribute('d', d);
}

gaitPeriodContainer.addEventListener('touchstart', startDragGaitPeriod);
gaitPeriodContainer.addEventListener('mousedown', startDragGaitPeriod);
function startDragGaitPeriod(event) {
    if (activeTouches.gaitPeriod != null) return;
    event.preventDefault();

    if (event.touches) {
        for (let i = 0; i < event.touches.length; i++) {
            const target = event.touches[i].target;
            if (target === gaitPeriodKnob || target === gaitPeriodContainer || target === gaitPeriodSVG || target === gaitPeriodTrackPath || target === gaitPeriodFillPath) {
                activeTouches.gaitPeriod = event.touches[i].identifier;
                break;
            }
        }
    } else {
        gaitPeriodActive = true;
    }
    updateGaitPeriodSliderFromEvent(event);
}
function updateGaitPeriodSliderFromEvent(event) {
    const rect = gaitPeriodContainer.getBoundingClientRect();
    let mouseY;
    if (event.type == 'touchmove') {
        for (let i = 0; i < event.touches.length; i++) {
            if (event.touches[i].identifier === activeTouches.gaitPeriod) {
                mouseY = event.touches[i].clientY - 25;
                break;
            }
        }
    }
    else {
        mouseY = event.clientY - 25;
    }
    if (!mouseY) {
        console.warn("Mouse Y coordinate not found, aborting update.");
        return;
    }
    const y = mouseY - rect.top;
    const progress = Math.max(0, Math.min(1, (gaitPeriodStartY - y) / gaitPeriodSliderHeight));
    gaitPeriodValue = progress;
    onParamsChanged();
    updateGaitPeriodSliderPosition(gaitPeriodValue);
    updateDebugInfo("gait-period-info", `Gait period slider updated: ${gaitPeriodValue.toFixed(2)}`);
}

const gaitLengthContainer = document.querySelector('.gait-length-container');
const gaitLengthSVG = document.getElementById('gait-length-svg');
const gaitLengthTrackPath = document.getElementById('gait-length-track-path');
const gaitLengthFillPath = document.getElementById('gait-length-fill-path');
const gaitLengthKnob = document.getElementById('gait-length-knob');
let gaitLengthValue = 0.5;
let gaitLengthActive = false;
const gaitLengthStartY = 180;
let gaitLengthSliderHeight = 160;
function initGaitLengthSlider() {
    updateGaitLengthSliderPosition(gaitLengthValue);
}

function updateGaitLengthSliderPosition(progress) {
    gaitLengthValue = progress;
    document.getElementById('gait-length-value').textContent = gaitLengthValue.toFixed(2);
    console.info(`Updating gait length slider position: ${gaitLengthValue}`);
    const knobPosition = gaitLengthStartY - progress * gaitLengthSliderHeight;

    gaitLengthKnob.style.top = `${knobPosition}px`;

    updateGaitLengthFillPath(progress);
}
function updateGaitLengthFillPath(progress) {
    const path = document.getElementById('gait-length-fill-path');
    const segments = 20;

    let d = `M 60, ${gaitLengthStartY}`;
    for (let i = 0; i <= segments; i++) {
        const p = Math.min(i / segments, progress);
        const y = gaitLengthStartY - p * gaitLengthSliderHeight;
        d += ` L${60},${y}`;
    }

    path.setAttribute('d', d);
}
gaitLengthContainer.addEventListener('touchstart', startDragGaitLength);
gaitLengthContainer.addEventListener('mousedown', startDragGaitLength);
function startDragGaitLength(event) {
    if (activeTouches.gaitLength != null) return;
    event.preventDefault();

    if (event.touches) {
        for (let i = 0; i < event.touches.length; i++) {
            const target = event.touches[i].target;
            if (target === gaitLengthKnob || target === gaitLengthContainer || target === gaitLengthSVG || target === gaitLengthTrackPath || target === gaitLengthFillPath) {
                activeTouches.gaitLength = event.touches[i].identifier;
                break;
            }
        }
    } else {
        gaitLengthActive = true;
    }
    updateGaitLengthSliderFromEvent(event);
}
function updateGaitLengthSliderFromEvent(event) {
    const rect = gaitLengthContainer.getBoundingClientRect();
    let mouseY;
    if (event.type == 'touchmove') {
        for (let i = 0; i < event.touches.length; i++) {
            if (event.touches[i].identifier === activeTouches.gaitLength) {
                mouseY = event.touches[i].clientY - 25;
                break;
            }
        }
    }
    else {
        mouseY = event.clientY - 25;
    }
    if (!mouseY) {
        console.warn("Mouse Y coordinate not found, aborting update.");
        return;
    }
    const y = mouseY - rect.top;
    const progress = Math.max(0, Math.min(1, (gaitLengthStartY - y) / gaitLengthSliderHeight));
    gaitLengthValue = progress;
    onParamsChanged();
    updateGaitLengthSliderPosition(gaitLengthValue);
    updateDebugInfo("gait-length-info", `Gait length slider updated: ${gaitLengthValue.toFixed(2)}`);
}

const zSwingContainer = document.querySelector('.z-swing-container');
const zSwingSVG = document.getElementById('z-swing-svg');
const zSwingTrackPath = document.getElementById('z-swing-track-path');
const zSwingFillPath = document.getElementById('z-swing-fill-path');
const zSwingKnob = document.getElementById('z-swing-knob');
let zSwingValue = 0.3;
let zSwingActive = false;
const zSwingStartX = 20;
let zSwingSliderWidth = 300;
function initZSwingSlider() {
    updateZSwingSliderPosition(zSwingValue);
}
function updateZSwingSliderPosition(progress) {
    zSwingValue = progress;
    document.getElementById('z-swing-value').textContent = zSwingValue.toFixed(2);
    console.info(`Updating Z-Swing slider position: ${zSwingValue}`);
    const knobPosition = progress * zSwingSliderWidth;

    zSwingKnob.style.left = `${knobPosition}px`;

    updateZSwingFillPath(progress);
}
function updateZSwingFillPath(progress) {
    const path = document.getElementById('z-swing-fill-path');
    const segments = 40;

    let d = `M ${zSwingStartX}, 50`;
    for (let i = 0; i <= segments; i++) {
        const p = Math.min(i / segments, progress);
        const x = zSwingStartX + p * zSwingSliderWidth;
        d += ` L${x},50`;
    }

    path.setAttribute('d', d);
}

zSwingContainer.addEventListener('touchstart', startDragZSwing);
zSwingContainer.addEventListener('mousedown', startDragZSwing);
function startDragZSwing(event) {
    if (activeTouches.ZSwing != null) return;
    event.preventDefault();

    if (event.touches) {
        for (let i = 0; i < event.touches.length; i++) {
            const target = event.touches[i].target;
            if (target === zSwingKnob || target === zSwingContainer || target === zSwingSVG || target === zSwingTrackPath || target === zSwingFillPath) {
                activeTouches.ZSwing = event.touches[i].identifier;
                break;
            }
        }
    } else {
        zSwingActive = true;
    }
    updateZSwingSliderFromEvent(event);
}
function updateZSwingSliderFromEvent(event) {
    const rect = zSwingContainer.getBoundingClientRect();
    let mouseX;
    if (event.type == 'touchmove') {
        for (let i = 0; i < event.touches.length; i++) {
            if (event.touches[i].identifier === activeTouches.ZSwing) {
                mouseX = event.touches[i].clientX;
                break;
            }
        }
    } else {
        mouseX = event.clientX;
    }
    if (!mouseX) {
        console.warn("Mouse X coordinate not found, aborting update.");
        return;
    }
    const x = mouseX - rect.left;
    const progress = Math.max(0, Math.min(1, (x - zSwingStartX) / zSwingSliderWidth));
    zSwingValue = progress;
    onParamsChanged();
    updateZSwingSliderPosition(zSwingValue);
    updateDebugInfo("z-swing-info", `Z-Swing slider updated: ${zSwingValue.toFixed(2)}`);
}

// 初始化
window.addEventListener('load', () => {
    initVerticalSlider();
    initPitchSlider();
    initGaitPeriodSlider();
    initGaitLengthSlider();
    initZSwingSlider();
})

// 摇杆元素
const leftJoystickOuter = document.getElementById('left-outer');
const leftJoystickInner = document.getElementById('left-inner');
const rightJoystickOuter = document.getElementById('right-outer');
const rightJoystickInner = document.getElementById('right-inner');
// 按钮元素
const resetButton = document.getElementById('reset-all');
const jumpButton = document.getElementById('jump-btn');
const flipButton = document.getElementById('flip-btn');
const navigationButton = document.getElementById('navigation-btn');
const crawButton = document.getElementById('craw-btn');
const sandpitButton = document.getElementById('sandpit-btn');
const straightupButton = document.getElementById('straightup-btn');
const stairsButton = document.getElementById('stairs-btn');
const slopeButton = document.getElementById('slope-btn');
const sideslopeButton = document.getElementById('sideslope-btn');

// 新版更新调试信息函数
// 在全局作用域定义消息管理对象
const messageRegistry = new Map();
// 重写消息显示函数
function updateDebugInfo(id, message) {
    const debugList = document.getElementById('debugList');
    const timestamp = `[${new Date().toLocaleTimeString()}]`;
    let msgElement = document.querySelector(`[data-id="${id}"]`); // 通过ID查找元素
    // 情况1：元素存在且已隐藏 → 重激活
    if (msgElement && msgElement.style.display === 'none') {
        msgElement.textContent = `${timestamp} ${message}`;
        msgElement.style.opacity = '1';
        msgElement.style.display = 'block'; // 重新显示
        clearTimeout(msgElement.timer); // 清除旧计时器
        msgElement.timer = startFadeTimer(msgElement, id); // 重启淡出计时
        messageRegistry.set(id, msgElement); // 重新注册
        return;
    }
    // 情况2：元素存在且显示中 → 更新内容并重置计时器
    if (msgElement) {
        msgElement.textContent = `${timestamp} ${message}`;
        clearTimeout(msgElement.timer);
        msgElement.timer = startFadeTimer(msgElement, id);
        return;
    }
    // 情况3：元素不存在 → 新建消息
    msgElement = document.createElement('div');
    msgElement.className = 'debug-item';
    msgElement.dataset.id = id; // 标记ID以便查找
    msgElement.textContent = `${timestamp} ${message}`;
    debugList.prepend(msgElement);
    msgElement.timer = startFadeTimer(msgElement, id); // 启动淡出计时
    messageRegistry.set(id, msgElement); // 注册新消息
    manageMessageQueue(); // 管理队列长度
    debugList.scrollTop = 0;
}
// 启动淡出计时器
function startFadeTimer(element, id) {
    return setTimeout(() => {
        element.style.transition = 'opacity 0.5s ease';
        element.style.opacity = '0';
        // 淡出后隐藏而非移除元素
        setTimeout(() => {
            element.style.display = 'none'; 
            messageRegistry.delete(id); // 清理注册表记录
        }, 500);
    }, 3000);
}
// 管理消息队列
function manageMessageQueue() {
    const messages = debugList.querySelectorAll('.debug-item');
    if (messages.length > 20) {
        // 移除最旧且未显示的消息（非激活状态）
        const oldestMsg = Array.from(messages).find(msg => 
            msg.style.display !== 'none'
        );
        oldestMsg?.remove();
    }
}

// 摇杆功能 - 支持多点触控
function setupJoystickEvents() {
    // 左侧摇杆触摸开始
    leftJoystickOuter.addEventListener('touchstart', function(e) {
        // 检查是否已有一个活动触摸点
        if (activeTouches.left !== null) return;
        
        const touch = e.changedTouches[0];
        activeTouches.left = touch.identifier;
        
        const rect = leftJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        // 更新摇杆位置
        updateJoystickPosition('left', touch.clientX - centerX, touch.clientY - centerY);
        
        e.preventDefault();
        updateDebugInfo("left-joystick","Left joystick touch start");
    }, { passive: false });
    
    // 右侧摇杆触摸开始
    rightJoystickOuter.addEventListener('touchstart', function(e) {
        // 检查是否已有一个活动触摸点
        if (activeTouches.right !== null) return;

        const touch = e.changedTouches[0];
        activeTouches.right = touch.identifier;
        
        const rect = rightJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        // 更新摇杆位置
        updateJoystickPosition('right', touch.clientX - centerX, touch.clientY - centerY);
        
        e.preventDefault();
        updateDebugInfo("right-joystick","Right joystick touch start");
    }, { passive: false });
    
    // 左侧摇杆鼠标按下
    leftJoystickOuter.addEventListener('mousedown', function(e) {
        activeTouches.left = 'mouse';
        const rect = leftJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        updateJoystickPosition('left', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    });
    
    // 右侧摇杆鼠标按下
    rightJoystickOuter.addEventListener('mousedown', function(e) {
        activeTouches.right = 'mouse';
        const rect = rightJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        updateJoystickPosition('right', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    });

    // 新增鼠标移动事件绑定
    leftJoystickOuter.addEventListener('mousemove', function(e) {
        if (activeTouches.left === 'mouse') {
            const rect = leftJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            updateJoystickPosition('left', e.clientX - centerX, e.clientY - centerY);
        }
    });

    rightJoystickOuter.addEventListener('mousemove', function(e) {
        if (activeTouches.right === 'mouse') {
            const rect = rightJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            updateJoystickPosition('right', e.clientX - centerX, e.clientY - centerY);
        }
    });
}

// 更新摇杆位置
function updateJoystickPosition(type, dx, dy) {
    const rect = (type === 'left' ? 
                  leftJoystickOuter : 
                  rightJoystickOuter).getBoundingClientRect();
    
    // 最大移动距离
    const maxDistance = Math.min(rect.width, rect.height) * 0.35;
    
    const distance = Math.sqrt(dx * dx + dy * dy);
    let newDx = dx;
    let newDy = dy;
    
    // 限制在最大距离内
    if (distance > maxDistance) {
        const scale = maxDistance / distance;
        newDx = dx * scale;
        newDy = dy * scale;
    }
    
    // 更新摇杆位置
    const element = (type === 'left' ? 
                     leftJoystickInner : 
                     rightJoystickInner);
    element.style.transform = `translate(calc(-50% + ${newDx}px), calc(-50% + ${newDy}px))`;
    
    // 存储输出值
    if (type === 'left') {
        leftOutput.x = -newDy / maxDistance;
        leftOutput.y = newDx / maxDistance;
        if (leftOutput.y * leftOutput.y + leftOutput.x * leftOutput.x < config.dead_zone * config.dead_zone) {
            leftOutput = { x: 0, y: 0 }; // 重置为0
        }
        else {
            // 应用死区处理
            const alpha = Math.atan2(leftOutput.y, leftOutput.x);
            leftOutput.y = (leftOutput.y - config.dead_zone * Math.sin(alpha)) / (1 - config.dead_zone) * Math.abs(Math.sin(alpha));
            leftOutput.x = (leftOutput.x - config.dead_zone * Math.cos(alpha)) / (1 - config.dead_zone) * Math.abs(Math.cos(alpha));
        }
        leftOutput.x = leftOutput.x > 0 ? Math.sqrt(leftOutput.x) : -Math.sqrt(-leftOutput.x);
        leftOutput.y = leftOutput.y > 0 ? Math.sqrt(leftOutput.y) : -Math.sqrt(-leftOutput.y);
        document.getElementById('left-x-output').textContent = leftOutput.x.toFixed(2);
        document.getElementById('left-y-output').textContent = leftOutput.y.toFixed(2);
        updateDebugInfo("left-info",`Left joystick: X:${leftOutput.x.toFixed(2)}, Y:${leftOutput.y.toFixed(2)}`);
    } else {
        rightOutput.x = newDx / maxDistance;
        if (rightOutput.x * rightOutput.x < config.dead_zone * config.dead_zone) {
            rightOutput = { x: 0 }; // 重置为0
        }
        else {
            rightOutput.x = (rightOutput.x - (rightOutput.x > 0 ? config.dead_zone : -config.dead_zone)) / (1 - config.dead_zone);
        }
        document.getElementById('right-x-output').textContent = rightOutput.x.toFixed(2);
        updateDebugInfo("right-info",`Right joystick: X:${rightOutput.x.toFixed(2)}`);
    }
}

function inQuad(progress){
    return progress * progress;
}

document.addEventListener('touchstart', function(e) {
    for (let i = 0; i < e.touches.length; i++) {
        const touch = e.touches[i];

        if (touch.target === resetButton) {
            resetButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("reset-button", "Reset button touched");
        }
        if (touch.target === jumpButton) {
            jumpButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("jump-button", "Jump button touched");
        }
        if (touch.target === flipButton) {
            flipButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("flip-button", "Flip button touched");
        }
        if (touch.target === stairsButton) {
            stairsButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("stairs-button", "Stairs button touched");
        }
        if (touch.target === slopeButton) {
            slopeButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("slope-button", "Slope button touched");
        }
        if (touch.target === sideslopeButton) {
            sideslopeButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("sideslope-button", "Side Slope button touched");
        }
        if (touch.target === crawButton) {
            crawButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("craw-button", "Craw button touched");
        }
        if (touch.target === sandpitButton) {
            sandpitButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("sandpit-button", "Sandpit button touched");
        }
        if (touch.target === straightupButton) {
            straightupButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("straightup-button", "Straight Up button touched");
        }
        if (touch.target === navigationButton) {
            navigationButton.dispatchEvent(new TouchEvent('click'));
            e.preventDefault();
            updateDebugInfo("navigation-button", "Navigation button touched");
        }
    }
})

// 触摸移动处理 - 多点触控支持
document.addEventListener('touchmove', function(e) {
    e.preventDefault();
    
    for (let i = 0; i < e.touches.length; i++) {
        const touch = e.touches[i];
        
        // 更新左侧摇杆
        if (activeTouches.left === touch.identifier) {
            const rect = leftJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            
            updateJoystickPosition('left', touch.clientX - centerX, touch.clientY - centerY);
        }
        
        // 更新右侧摇杆
        if (activeTouches.right === touch.identifier) {
            const rect = rightJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            
            updateJoystickPosition('right', touch.clientX - centerX, touch.clientY - centerY);
        }
        
        // 更新高度条
        if (activeTouches.vertical === touch.identifier) {
            updateVerticalSliderFromEvent(e);
        }
        // 更新倾角条
        if (activeTouches.pitch === touch.identifier) {
            updatePitchSliderFromEvent(e);
        }

        if (activeTouches.gaitLength === touch.identifier) {
            updateGaitLengthSliderFromEvent(e);
        }

        if (activeTouches.gaitPeriod === touch.identifier) {
            updateGaitPeriodSliderFromEvent(e);
        }

        // 更新Z-Swing条
        if (activeTouches.ZSwing === touch.identifier) {
            updateZSwingSliderFromEvent(e);
        }
    }
}, { passive: false });

// 触摸结束处理
document.addEventListener('touchend', function(e) {
    for (let i = 0; i < e.changedTouches.length; i++) {
        const touch = e.changedTouches[i];
        
        // 重置左侧摇杆
        if (activeTouches.left === touch.identifier) {
            resetJoystick('left');
            activeTouches.left = null;
            updateDebugInfo("left-joystick","Left joystick touch end");
        }
        
        // 重置右侧摇杆
        if (activeTouches.right === touch.identifier) {
            resetJoystick('right');
            activeTouches.right = null;
            updateDebugInfo("right-joystick","Right joystick touch end");
        }
        
        // 重置高度条
        if (activeTouches.vertical === touch.identifier) {
            verticalActive = false;
            activeTouches.vertical = null;
            updateDebugInfo("height-info","Height slider touch end");
        }

        // 重置倾角条
        if (activeTouches.pitch === touch.identifier) {
            pitchActive = false;
            activeTouches.pitch = null;
            updateDebugInfo("pitch-info", "Pitch slider touch end");
        }

        // 重置步态周期条
        if (activeTouches.gaitPeriod === touch.identifier) {
            gaitPeriodActive = false;
            activeTouches.gaitPeriod = null;
            updateDebugInfo("gait-period-info", "Gait period slider touch end");
        }

        // 重置步态长度条
        if (activeTouches.gaitLength === touch.identifier) {
            gaitLengthActive = false;
            activeTouches.gaitLength = null;
            updateDebugInfo("gait-length-info", "Gait length slider touch end");
        }

        // 重置Z-Swing条
        if (activeTouches.ZSwing === touch.identifier) {
            zSwingActive = false;
            activeTouches.ZSwing = null;
            updateDebugInfo("z-swing-info", "Z-Swing slider touch end");
        }
    }
});

// 鼠标移动处理
document.addEventListener('mousemove', function(e) {
    if (verticalActive) {
        updateVerticalSliderFromEvent(e);
    }
    
    if (activeTouches.left === 'mouse') {
        const rect = leftJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        updateJoystickPosition('left', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    }
    
    if (activeTouches.right === 'mouse') {
        const rect = rightJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        updateJoystickPosition('right', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    }
    if (pitchActive) {
        updatePitchSliderFromEvent(e);
    }

    if (gaitPeriodActive) {
        updateGaitPeriodSliderFromEvent(e);
    }

    if (gaitLengthActive) {
        updateGaitLengthSliderFromEvent(e);
    }

    if (zSwingActive) {
        updateZSwingSliderFromEvent(e);
    }

});

// 鼠标抬起处理
document.addEventListener('mouseup', function() {
    if (verticalActive) {
        verticalActive = false;
        activeTouches.vertical = null;
        updateDebugInfo("height-info","Height slider mouse up");
    }
    
    if (activeTouches.left === 'mouse') {
        resetJoystick('left');
        activeTouches.left = null;
    }
    
    if (activeTouches.right === 'mouse') {
        resetJoystick('right');
        activeTouches.right = null;
    }
    if (pitchActive) {
        pitchActive = false;
        activeTouches.pitch = null;
        updateDebugInfo("pitch-info", "Pitch slider mouse up");
    }
    if (gaitPeriodActive) {
        gaitPeriodActive = false;
        activeTouches.gaitPeriod = null;
        updateDebugInfo("gait-period-info", "Gait period slider mouse up");
    }
    if (gaitLengthActive) {
        gaitLengthActive = false;
        activeTouches.gaitLength = null;
        updateDebugInfo("gait-length-info", "Gait length slider mouse up");
    }
    if (zSwingActive) {
        zSwingActive = false;
        activeTouches.ZSwing = null;
        updateDebugInfo("z-swing-info", "Z-Swing slider mouse up");
    }
});

// 重置摇杆位置
function resetJoystick(type) {
    const element = (type === 'left' ? 
                     leftJoystickInner : 
                     rightJoystickInner);
    element.style.transform = 'translate(-50%, -50%)';
    
    if (type === 'left') {
        leftOutput = { x: 0, y: 0 };
        document.getElementById('left-x-output').textContent = "0.00";
        document.getElementById('left-y-output').textContent = "0.00";
    } else {
        rightOutput = { x: 0 };
        document.getElementById('right-x-output').textContent = "0.00";
    }
}
let config = {
    rosbridge_ip: "192.168.31.248",
    rosbridge_port: "9090",
    http_port: 8080,
    publish_interval: 10,
    dead_zone: 0.2, // 死区配置
    _isFallBack: false,
}
let buttonInterval;
// 重置所有控制器
function resetAllControls() {
    // 重置摇杆
    resetJoystick('left');
    resetJoystick('right');
    
    // 重置高度滑块
    verticalValue = 0.5;
    initVerticalSlider();
    document.getElementById('vertical-value').textContent = '0.50';
    
    // 重置倾角滑块
    pitchValue = 0.5;
    initPitchSlider();
    document.getElementById('pitch-value').textContent = '0.50';

    // 重置步态周期滑块
    gaitPeriodValue = 0.3;
    initGaitPeriodSlider();
    document.getElementById('gait-period-value').textContent = '0.30';

    // 重置步态长度滑块
    gaitLengthValue = 0.5;
    initGaitLengthSlider();
    document.getElementById('gait-length-value').textContent = '0.50';

    // 重置Z-Swing滑块
    zSwingValue = 0.3;
    initZSwingSlider();
    document.getElementById('z-swing-value').textContent = '0.30';
    
    // 重置输出值
    leftOutput = { x: 0, y: 0 };
    rightOutput = { x: 0 };

    triggerButton('recover'); // 触发恢复状态
    
    updateDebugInfo("sys-reset", "all controls have been reset");
}
let params = [0.5,0.5,0.3,0.5,0.3];
function onParamsChanged(){
    if (!paramsTopic || !connected) {
        updateDebugInfo("params-error", "Not connected to ROS or params topic not initialized");
        return;
    }
    params = [
        verticalValue,
        pitchValue,
        gaitPeriodValue,
        gaitLengthValue,
        zSwingValue
    ];
    const paramsMsg = new ROSLIB.Message({
        data: params
    });
    paramsTopic.publish(paramsMsg);
    updateDebugInfo("params-publish", `Publish params command ${params}`);
}
function triggerButton(buttonName) {
    if (!stateTopic || !connected) {
        updateDebugInfo("state-error", "Not connected to ROS or state topic not initialized");
        return;
    }
    let stateIndex;
    switch (buttonName) {
        case 'flip':
            stateIndex = 1;
            break;
        case 'jump':
            stateIndex = 2;
            break;
        case 'recover':
            stateIndex = 3;
            break;
        case 'navigation':
            stateIndex = 4;
            break;
        case 'craw':
            stateIndex = 5;
            break;
        case 'sandpit':
            stateIndex = 6;
            break;
        case 'straightup':
            stateIndex = 7;
            break;
        case 'stairs':
            stateIndex = 8;
            break;
        case 'slope':
            stateIndex = 9;
            break;
        case 'sideslope':
            stateIndex = 10;
            break;
        case null:
        default:
            stateIndex = 0; // 默认状态
            break;
    }
    const stateMsg = new ROSLIB.Message({
        data: stateIndex
    });
    stateTopic.publish(stateMsg);
    document.getElementById('message-count').textContent =
        parseInt(document.getElementById('message-count').textContent) + 1;

    updateDebugInfo("state-publish", `Publish state command ${buttonName}`);
}

// 页面加载完成后初始化
window.addEventListener('DOMContentLoaded', async () => {
    const LoadedConfig = await loadConfig();
    if (LoadedConfig._isFallBack) {
        updateDebugInfo("ros-config",'Using default config: ROS ' + LoadedConfig.rosbridge_ip + ':' + config.rosbridge_port + ', HTTP port: ' + config.http_port + ', publish frequency: ' + config.publish_interval + 'ms , dead zone size: ' + config.dead_zone);
    } else {
        updateDebugInfo("ros-config",'Config loading successfully: ROS ' + config.rosbridge_ip + ':' + LoadedConfig.rosbridge_port + ', HTTP port ' + LoadedConfig.http_port + ', publish frequency ' + LoadedConfig.publish_interval + 'ms , dead zone size: ' + LoadedConfig.dead_zone);
    }
    config = LoadedConfig; // 更新全局配置
    
    // 设置事件监听
    setupJoystickEvents();
    
    // 按钮事件
    document.getElementById('connectBtn').addEventListener('click', function() {
        startConnection();
        updateDebugInfo("ros-connection",'Trying to connect to ROS server...');
    });
    
    document.getElementById('disconnectBtn').addEventListener('click', function() {
        if (ros) {
            ros.close();
            updateDebugInfo("ros-connection",'Disconnected from ROS server');
            document.getElementById('connectBtn').textContent = 'Connect';
            document.getElementById('disconnectBtn').textContent = 'Disconnect';
            document.getElementById('ros-status').textContent = 'Not connected';
            document.getElementById('ros-indicator').className = 'indicator disconnected';
            connected = false;
        } else {
            updateDebugInfo("ros-connection",'No active ROS connection');
        }
    });
    
    // 初始状态
    updateDebugInfo("sys-info",'System initialized');
    
    // 窗口大小变化时重新计算位置
    window.addEventListener('resize', function() {
        initVerticalSlider();
        initPitchSlider();
        updateDebugInfo("sys-info",'Recalculating positions');
    });
    // 在初始化部分添加防抖的resize处理
    let resizeTimer;
    window.addEventListener('resize', function() {
        clearTimeout(resizeTimer);
        resizeTimer = setTimeout(() => {
            initVerticalSlider();
            updateDebugInfo("sys-info",'Window resize complete');
        }, 500); // 500ms防抖
    });
    // 添加重置按钮事件
    document.getElementById('reset-all').addEventListener('click', resetAllControls);
    
    // 添加上升按钮事件（第二个按钮改为起跳功能）
    const jumpBtn = document.getElementById('jump-btn');
    jumpBtn.addEventListener('click', () => triggerButton('jump'));

    const flipBtn = document.getElementById('flip-btn');
    flipBtn.addEventListener('click', () => triggerButton('flip'));

    const crawBtn = document.getElementById('craw-btn');
    crawBtn.addEventListener('click', () => triggerButton('craw'));

    const sandpitBtn = document.getElementById('sandpit-btn');
    sandpitBtn.addEventListener('click', () => triggerButton('sandpit'));

    const straightupBtn = document.getElementById('straightup-btn');
    straightupBtn.addEventListener('click', () => triggerButton('straightup'));

    const stairsBtn = document.getElementById('stairs-btn');
    stairsBtn.addEventListener('click', () => triggerButton('stairs'));

    const slopeBtn = document.getElementById('slope-btn');
    slopeBtn.addEventListener('click', () => triggerButton('slope'));

    const sideslopeBtn = document.getElementById('sideslope-btn');
    sideslopeBtn.addEventListener('click', () => triggerButton('sideslope'));

    const navigationBtn = document.getElementById('navigation-btn');
    navigationBtn.addEventListener('click', () => triggerButton('navigation'));
});
let ros = null;
let cmdVel = null;
let height = null;
let paramsTopic = null;
let stateTopic = null;
let connected = false;

function startConnection() {
    if (ros) {
        ros.close();
        ros = null;
    }
    ros = new ROSLIB.Ros({
        url: `ws://${config.rosbridge_ip}:${config.rosbridge_port}`
    });
    ros.on('connection', () => {
        console.log('Connected to ROS Bridge');
        cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
        });
        height = new ROSLIB.Topic({
            ros: ros,
            name: '/height',
            messageType: 'std_msgs/Float32'
        });
        paramsTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/params',
            messageType: 'std_msgs/Float32MultiArray'
        });
        stateTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/state',
            messageType: 'std_msgs/Int8'
        });
        updateDebugInfo("ros-connection",'Connected to ROS server');
        document.getElementById('connectBtn').textContent = 'Connected';
        document.getElementById('disconnectBtn').textContent = 'Disconnect';
        document.getElementById('ros-status').textContent = 'Connected';
        document.getElementById('ros-indicator').className = 'indicator connected';
        connected = true;
        paramsTopic.publish(new ROSLIB.Message({ data: params }));
        stateTopic.publish(new ROSLIB.Message({ data: 0 })); // 发布初始状态

    });
    ros.on('error', (error) => {
        console.error('Error connecting to ROS: ', error);
        alert('Error connecting to ROS: 9090', error);
    });
    ros.on('close', () => {
        console.log('Disconnected from ROS');
        updateDebugInfo("ros-connection",'Disconnected from ROS server');
    });
}
function move(){
    if (!ros) {
        console.error('ROS connection not established');
        alert('ROS connection not established');
        return;
    }
    if (!cmdVel) {
        console.error('Publisher not created');
        alert('Publisher not created');
        return;
    }
    if (!height) {
        console.error('Height publisher not created');
        alert('Height publisher not created');
        return;
    }
    const twist = new ROSLIB.Message({
        linear: {
            x: leftOutput.x,
            y: leftOutput.y,
            z: 0
        },
        angular: {
            x: 0,
            y: pitchValue, // 假设高度控制影响y轴
            z: -rightOutput.x // 假设右摇杆控制角速度
        }
    });
    const heightMsg = new ROSLIB.Message({
        data: verticalValue // 使用高度输出值
    });
    // 发布消息
    console.log(`Publishing: linear x=${twist.linear.x}, y=${twist.linear.y}, angular y=${twist.angular.y}, z=${twist.angular.z}`);
    cmdVel.publish(twist);
    height.publish(heightMsg);
    document.getElementById('message-count').textContent = parseInt(document.getElementById('message-count').textContent) + 1;
}
// 定时发布消息
const interval = setInterval(() => {
    if (connected) {
        move();
    }
}, config.publish_interval);