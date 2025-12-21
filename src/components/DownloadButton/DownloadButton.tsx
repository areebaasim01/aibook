import React from 'react';
import styles from './DownloadButton.module.css';

interface DownloadButtonProps {
    href: string;
    filename: string;
    description?: string;
    icon?: string;
}

export default function DownloadButton({
    href,
    filename,
    description,
    icon = '📥'
}: DownloadButtonProps) {
    return (
        <a
            href={href}
            download={filename}
            className={styles.downloadButton}
        >
            <span className={styles.icon}>{icon}</span>
            <div className={styles.content}>
                <span className={styles.filename}>{filename}</span>
                {description && (
                    <span className={styles.description}>{description}</span>
                )}
            </div>
            <span className={styles.downloadIcon}>⬇️</span>
        </a>
    );
}
