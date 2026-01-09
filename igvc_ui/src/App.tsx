import { useState } from 'react'
import { ImageView } from './components/ImageView.'
import { frontView$, yoloView$ } from './lib/rx/listeners/images'
import { addVisionCapability } from './lib/protocol/commands/capabilities'
import { VisonCapabilities } from './lib/protocol/capabilities'

function App() {
    const [count, setCount] = useState(0)

    return (
        <>
            <h1>Vite + React</h1>
            <div className="">
                <button onClick={() => setCount((count) => count + 1)}>
                    count is {count}
                </button>
                <p>
                    Edit <code>src/App.jsx</code> and save to test HMR
                </p>

                <ImageView stream$={frontView$} />
                <ImageView stream$={yoloView$} />

                <button onClick={() => {
                    addVisionCapability(VisonCapabilities.FrontCamera);
                }} className="m-2 p-2 bg-blue-500 text-white rounded-md hover:bg-blue-600">
                    Request Front Camera
                </button>

                 <button onClick={() => {
                    addVisionCapability(VisonCapabilities.YoloView);
                }} className="m-2 p-2 bg-blue-500 text-white rounded-md hover:bg-blue-600">
                    Request Yolo Camera
                </button>
            </div>
            <p className="read-the-docs">
                Click on the Vite and React logos to learn more
            </p>
        </>
    )
}

export default App
