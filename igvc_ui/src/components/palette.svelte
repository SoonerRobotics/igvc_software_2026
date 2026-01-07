<script>

import CommandPalette, { defineActions, createStoreMethods } from "svelte-command-palette";

const methods = createStoreMethods();

const pages = [
    { name: "Dashboard", path: "/dashboard" },
    { name: "Vision", path: "/vision" },
    { name: "Graphs", path: "/graphs" },
    { name: "Mapping", path: "/mapping" },
    { name: "Self Drive", path: "/self-drive" },
    { name: "Configuration", path: "/configuration" },
    { name: "Logs", path: "/logs" },
];
const actions = defineActions([
    {
        title: "Say hello",
        subTitle: "This action will print \"hello\" to the console",
        onRun: () => {
            console.log("hello");
        },
        shortcut: "$mod+d",
        keywords: ["home", "main"],
        group: "General"
    },

    // Pages
    ...pages.map((page, idx) => ({
        title: `Go to ${page.name}`,
        subTitle: `Navigate to \`${page.name}\``,
        onRun: () => {
            window.location.href = page.path;
        },
        shortcut: `$mod+${idx + 1}`,
        keywords: [page.name.toLowerCase()],
        group: "Pages"
    })),
])

</script>

<CommandPalette
    commands={actions}
    placeholder="Type a command..."
    shortcut="$mod+k"
/>

