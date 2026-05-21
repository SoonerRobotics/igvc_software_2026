import { Modal, Button, Group, Text, Badge } from '@mantine/core';

export type ConfigurationModalProps = {
    open: boolean;
    onClose?: () => void;
}

export default function ConfigurationModal(props: ConfigurationModalProps) {
    return (
        <>
            <Modal opened={props.open} onClose={() => props.onClose?.()} title="Configuration">
                <Text>Modal with size auto will fits its content</Text>

                <Group wrap="nowrap" mt="md">
                    <Badge color="pink" size="xl">Pink badge</Badge>
                    <Badge color="cyan" size="xl">Cyan badge</Badge>
                    <Badge color="grape" size="xl">Grape badge</Badge>
                </Group>
            </Modal>
        </>
    );
}