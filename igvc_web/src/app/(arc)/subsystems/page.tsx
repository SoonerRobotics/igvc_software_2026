import { Accordion, AccordionContent, AccordionItem, AccordionTrigger } from "@/components/ui/accordion";
import { Badge } from "@/components/ui/badge";

type SubsystemItemProps = {
    name: string;
    status: string;
    properties: {
        key: string;
        values: {
            value: string;
            timestamp: string;
        }[]
    }[]
}

function SubsystemItem(props: SubsystemItemProps) {
    return (
        <AccordionItem value={props.name}>
            <AccordionTrigger>
                <div className="flex w-full items-center justify-between">
                    <div className="flex items-center gap-2">
                        <h3 className="font-semibold">
                            {props.name}
                        </h3>
                        <Badge variant={"success"}>
                            {props.status}
                        </Badge>
                    </div>
                    <div className="flex flex-col items-end gap-1">
                        <div className="text-right text-xs text-white">
                            {props.properties.length} Properties
                        </div>
                        <div className="text-right text-xs text-white">
                            Last Updated: {props.properties.reduce((latest, property) => {
                                const latestValue = property.values.reduce((latestValue, value) => {
                                    return new Date(value.timestamp) > new Date(latestValue.timestamp) ? value : latestValue;
                                }, property.values[0]);
                                return new Date(latest) > new Date(latestValue.timestamp) ? latest : latestValue.timestamp;
                            }, props.properties[0].values[0].timestamp)}
                        </div>
                    </div>
                </div>
            </AccordionTrigger>
            <AccordionContent>
                <div className="flex flex-col gap-4">
                    {props.properties.map(property => (
                        <div key={property.key} className="flex flex-col gap-2">
                            <h4 className="text-sm font-medium text-white">
                                {property.key}
                            </h4>
                            <div className="flex flex-col gap-1">
                                {property.values.map((value, index) => (
                                    <div key={index} className="flex items-center justify-between text-xs text-white">
                                        <span>{value.value}</span>
                                        <span>{new Date(value.timestamp).toLocaleTimeString()}</span>
                                    </div>
                                ))}
                            </div>
                        </div>
                    ))}
                </div>
            </AccordionContent>
        </AccordionItem>
    )
}

export default function SubsystemsPage() {
    return (
        <div className="flex flex-col">
            <h1 className="text-2xl font-semibold text-white">
                Subsystems
            </h1>

            <div className="mt-4">
                <Accordion type="multiple" className="w-full">
                    <SubsystemItem
                        name="Drive"
                        status="Operational"
                        properties={[
                            {
                                key: "Speed",
                                values: [
                                    { value: "1.5 m/s", timestamp: "2024-06-01T12:00:00Z" },
                                    { value: "1.7 m/s", timestamp: "2024-06-01T12:01:00Z" },
                                ]
                            },
                            {
                                key: "Battery",
                                values: [
                                    { value: "80%", timestamp: "2024-06-01T12:00:00Z" },
                                    { value: "78%", timestamp: "2024-06-01T12:01:00Z" },
                                ]
                            }
                        ]}
                    />
                </Accordion>
            </div>
        </div>
    )
}