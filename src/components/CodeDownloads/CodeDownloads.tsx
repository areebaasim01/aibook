import React from 'react';
import DownloadButton from '../DownloadButton';
import styles from './CodeDownloads.module.css';

interface CodeFile {
    filename: string;
    description: string;
    icon?: string;
}

interface CodeDownloadsProps {
    module: 1 | 2 | 3 | 4;
    files: CodeFile[];
    title?: string;
}

export default function CodeDownloads({
    module,
    files,
    title = '📦 Download Code Examples'
}: CodeDownloadsProps) {
    const basePath = `/ai-native-book/code-examples/module-${module}`;

    return (
        <div className={styles.container}>
            <h3 className={styles.title}>{title}</h3>
            <div className={styles.grid}>
                {files.map((file, index) => (
                    <DownloadButton
                        key={index}
                        href={`${basePath}/${file.filename}`}
                        filename={file.filename}
                        description={file.description}
                        icon={file.icon || getIconForFile(file.filename)}
                    />
                ))}
            </div>
        </div>
    );
}

function getIconForFile(filename: string): string {
    const ext = filename.split('.').pop()?.toLowerCase();
    switch (ext) {
        case 'py':
            return '🐍';
        case 'urdf':
        case 'xacro':
            return '🤖';
        case 'yaml':
        case 'yml':
            return '⚙️';
        case 'sdf':
        case 'world':
            return '🌍';
        case 'launch':
            return '🚀';
        default:
            return '📄';
    }
}
